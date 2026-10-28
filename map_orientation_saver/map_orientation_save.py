"""
Save a Nav2 raster map together with matching geographic orientation metadata.

The command subscribes to localization/map_orientation, captures a fresh valid
estimate if available, and invokes Nav2's map_saver_cli. After a successful
raster save it checks estimator continuity and atomically writes a parallel
<map>.orientation.yaml file, bound to the saved YAML/image bytes by SHA-256.
It needs no separately running map-saver server and does not serialize SLAM
posegraphs, estimate heading itself, or detect saves made by another tool.

Usage while a map publisher and orientation estimator are running:
    ros2 run map_orientation_saver map_saver_with_orientation \
        -f ~/launch/maps/backyard

Nav2 CLI options and ROS arguments are forwarded. To save in simulation, add
`--ros-args -p use_sim_time:=true`; select a namespace with `-r __ns:=/robot1`.
Use --help for wrapper-specific freshness, topic, frame, and timeout options.
The package includes config/map_saver_params.yaml for Nav2 save parameters.

A missing orientation does not prevent raster saving: the sidecar records
valid=false and a reason. Nav2 or sidecar-write failures exit nonzero. A saved
raster is not rolled back if writing its sidecar fails; readers must validate
the sidecar content hash before associating it with a map.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
import signal
import stat
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import yaml


def map_fingerprint(map_yaml):
    """Read the saved YAML and referenced image and compute their content hash.

    Relative image paths are resolved against the YAML directory. The digest
    covers YAML bytes, one NUL separator, then image bytes, matching the
    sidecar/map identity convention used by downstream coverage tools.
    """
    path = Path(map_yaml)
    yaml_bytes = path.read_bytes()
    settings = yaml.safe_load(yaml_bytes)
    image = Path(settings['image']).expanduser()
    if not image.is_absolute():
        image = path.parent/image
    digest = hashlib.sha256(yaml_bytes+b'\0'+image.read_bytes()).hexdigest()
    return digest, settings


def fresh_estimate(estimate, now_ns, max_age, map_frame):
    """Check validity, true-North reference, expected map frame, finite angles, and clock freshness.
    """
    try:
        return bool(
            estimate['valid'] is True and estimate['reference'] == 'true_north'
            and estimate['map_frame'] == map_frame
            and 0 <= (now_ns-int(estimate['published_at_ros_ns']))/1e9 <= max_age
            and 0 <= now_ns/1e9-estimate['stamp'] <= max_age
            and all(math.isfinite(estimate[k]) for k in
                    ('map_x_heading_from_east_deg','uncertainty_deg')))
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def write_sidecar(map_yaml, estimate, reason=''):
    """Atomically bind an orientation snapshot or unavailable reason to a saved map.

    Read/hash both raster files first, then write and fsync a temporary YAML
    in the same directory before replacement. Failed validation leaves an
    existing sidecar untouched; a successful write replaces old metadata.
    The temporary file inherits the map YAML's access permissions before the
    atomic rename, so group/other readers see the same access as the map.
    """
    path = Path(map_yaml)
    fingerprint, settings = map_fingerprint(path)
    body = dict(schema_version=1, map_file=path.name, map_content_hash=fingerprint,
                saved_at=datetime.now(timezone.utc).isoformat(),
                geometry={k:settings.get(k) for k in ('resolution','origin')},
                orientation=estimate if estimate else dict(valid=False,reason=reason))
    sidecar = path.with_suffix('.orientation.yaml')
    temp = None
    try:
        with tempfile.NamedTemporaryFile(mode='w',dir=path.parent,prefix='.'+sidecar.name,
                                         suffix='.tmp',delete=False) as stream:
            temp = Path(stream.name)
            yaml.safe_dump(body,stream,sort_keys=False)
            stream.flush()
            # NamedTemporaryFile starts at 0600 regardless of the map/umask.
            # Copy ordinary access bits, without propagating special mode bits.
            os.fchmod(stream.fileno(), stat.S_IMODE(path.stat().st_mode) & 0o777)
            os.fsync(stream.fileno())
        os.replace(temp,sidecar)
    finally:
        if temp is not None:
            temp.unlink(missing_ok=True)
    return sidecar


def nav2_arguments(raw, filename):
    """Remove wrapper-only flags while preserving Nav2 options, parameter files, and ROS remaps.
    """
    own = ('--orientation-topic','--orientation-timeout','--orientation-max-age',
           '--orientation-map-frame','--save-timeout')
    result = ['-f',str(filename)]
    i = 0
    while i < len(raw):
        token = raw[i]
        if token == '-f' or token in own:
            i += 2
        elif any(token.startswith(name+'=') for name in own):
            i += 1
        elif token.startswith('-f') and not token.startswith('--'):
            i += 1
        else:
            result.append(token)
            i += 1
    return result


def main(args=None):
    """Run the map-saving CLI workflow and return a nonzero exit on failure.

    Capture an estimate, invoke Nav2, monitor completion, verify that the
    orientation session/segment remained valid, then write the sidecar.
    The Nav2 subprocess group is terminated if timeout or cleanup requires it.
    """
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rclpy.utilities import remove_ros_args
    from std_msgs.msg import String
    raw = list(sys.argv[1:] if args is None else args)
    parser = argparse.ArgumentParser(description=__doc__,epilog='Other arguments are forwarded to Nav2 map_saver_cli.')
    parser.add_argument('-f',required=True,help='Output map basename (without .yaml)')
    parser.add_argument('--orientation-topic',default='localization/map_orientation')
    parser.add_argument('--orientation-timeout',type=float,default=3.)
    parser.add_argument('--orientation-max-age',type=float,default=2.)
    parser.add_argument('--orientation-map-frame',default='map')
    parser.add_argument('--save-timeout',type=float,default=30.)
    options, _ = parser.parse_known_args(remove_ros_args(args=['map_saver_with_orientation',*raw])[1:])
    for name in ('orientation_timeout','orientation_max_age','save_timeout'):
        if not math.isfinite(getattr(options,name)) or getattr(options,name) <= 0:
            parser.error(name.replace('_','-')+' must be finite and positive')
    filename = Path(options.f).expanduser().absolute()
    if filename.suffix == '.yaml':
        filename = filename.with_suffix('')
    filename.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init(args=['map_saver_with_orientation',*raw])
    node = None
    process = None
    try:
        node = rclpy.create_node('map_saver_with_orientation')
        latest = [None]
        def receive(message):
            try:
                decoded = json.loads(message.data)
                latest[0] = decoded if isinstance(decoded,dict) else None
            except (ValueError, TypeError):
                latest[0] = None
        subscription = node.create_subscription(String,options.orientation_topic,receive,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        def fresh(value):
            return fresh_estimate(value,node.get_clock().now().nanoseconds,
                                  options.orientation_max_age,options.orientation_map_frame)
        deadline = time.monotonic()+options.orientation_timeout
        while time.monotonic() < deadline and not fresh(latest[0]):
            rclpy.spin_once(node,timeout_sec=.1)
        captured = latest[0] if fresh(latest[0]) else None
        command = ['ros2','run','nav2_map_server','map_saver_cli',*nav2_arguments(raw,filename)]
        node.get_logger().info(f'Saving map to {filename}.yaml')
        process = subprocess.Popen(command, start_new_session=True)
        deadline = time.monotonic()+options.save_timeout
        while process.poll() is None:
            if time.monotonic() >= deadline:
                raise TimeoutError('Nav2 map saver timed out; no orientation sidecar written')
            rclpy.spin_once(node,timeout_sec=.1)
        if process.returncode != 0:
            raise RuntimeError(f'Nav2 map saver failed with exit code {process.returncode}; no sidecar written')
        reason = 'No fresh valid orientation estimate available at save request'
        if captured is not None:
            current = latest[0]
            same_segment = fresh(current) and all(current.get(k)==captured.get(k)
                for k in ('session_id','segment','map_frame'))
            if not same_segment:
                captured = None
                reason = 'Orientation became unavailable or reset during map saving'
        sidecar = write_sidecar(Path(str(filename)+'.yaml'),captured,reason)
        node.get_logger().info(f'Orientation metadata saved: {sidecar}')
        if captured is None:
            node.get_logger().warning(reason)
        node.destroy_subscription(subscription)
    except (OSError, ValueError, RuntimeError, TimeoutError, yaml.YAMLError) as error:
        print(f'map_saver_with_orientation: {error}',file=sys.stderr)
        raise SystemExit(1) from error
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
