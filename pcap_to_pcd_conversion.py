"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Executable examples for using the pcap APIs.

This module has a rudimentary command line interface. For usage, run::

    $ python -m ouster.sdk.examples.pcap -h
"""


"""
Command to execute:

python3 00_pcap1.0_time_adjustment.py /mnt/DataDrive/Recordings/AOZ/Day/ToStage3_FW/Dynamic/20240328_112627/lidar/lidar_FF/FrontForward.pcap /mnt/DataDrive/Recordings/AOZ/Day/ToStage3_FW/Dynamic/20240328_112627/lidar/lidar_FF/FrontForward.json pcap-to-pcd -o /mnt/DataDrive/Recordings/AOZ/Day/ToStage3_FW/Dynamic/20240328_112627/lidar/lidar_FF/PCD

"""



import os
import argparse
from contextlib import closing
import numpy as np
from pypcd4 import PointCloud, Encoding
from ouster.sdk import client, pcap

fields = ("x", "y", "z", "intensity")
types = (np.float32, np.float32, np.float32, np.uint8)

def append_timestamp_to_pcd_header(pcd_path, timestamp):
    # Read the existing content of the PCD file
    with open(pcd_path, 'r') as f:
        lines = f.readlines()

    # Append the TIMESTAMP line with the specified timestamp
    timestamp_line = f"TIMESTAMP {timestamp}\n"
    lines.insert(10, timestamp_line)  # Insert at line 10, adjust if needed

    # Write the modified content back to the PCD file
    with open(pcd_path, 'w') as f:
        f.writelines(lines)

def pcap_to_pcd(source: client.PacketSource,
                metadata: client.SensorInfo,
                num: int = 0,
                output_dir: str = "./output"): # Default directory path for output csv and pcd"

    if (metadata.format.udp_profile_lidar ==
            client.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL):
        print("Note: You've selected to convert a dual returns pcap. Second "
              "returns are ignored in this conversion by this example "
              "for clarity reasons.  You can modify the code as needed by "
              "accessing it through github or the SDK documentation.")

    from itertools import islice

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    try:
        columns_per_frame = metadata.format.columns_per_frame
        pixels_per_column = metadata.format.pixels_per_column
    except Exception as e:
        print(f'Error parsing metadata: {e}')

    # precompute xyzlut to save computation in a loop
    xyzlut = client.XYZLut(metadata)

    # create an iterator of LidarScans from pcap and bound it if num is specified

    if num:
        scans = islice(scans, num)
    log_file = os.path.join(output_dir, "zero_timestamp_logs.txt")

    scans = iter(client.Scans(source))

    with open(log_file, "w") as f:
        for idx, scan in enumerate(scans):
            xyz = xyzlut(scan.field(client.ChanField.RANGE))
            key = scan.field(client.ChanField.REFLECTIVITY)
            key_expanded = np.expand_dims(key, axis=-1)
            np.set_printoptions(threshold=np.inf)
            merged_array = np.concatenate((xyz, key_expanded), axis=-1)
            merged_array_reshaped = merged_array.reshape(pixels_per_column*columns_per_frame, 4)

            # Filter points where x, y, or z coordinates are zero
            non_zero_indices = np.where(np.all(merged_array_reshaped[:, :3] != 0, axis=1))
            merged_array_reshaped = merged_array_reshaped[non_zero_indices]
            #print (merged_array_reshaped)
            # Check if there are any points left after filtering
            if len(merged_array_reshaped) == 0:
                print(f"No valid points in frame #{idx}. Skipping...")
                continue

            pc = PointCloud.from_points(merged_array_reshaped, fields, types)
            pcd_path = os.path.join(output_dir, f'pcd_out_{idx:06d}.pcd')
            print(f'write frame #{idx} to file: {pcd_path}')
            is_zero = np.any(scan.timestamp == 0)
            if is_zero:
                zero_blocks = []
                start_block = None
                prev_block = None
                for idx2, status in enumerate(scan.status):
                    if status == 0:
                        if start_block is None:
                            start_block = idx2
                        prev_block = idx2
                    elif start_block is not None:
                        zero_blocks.append((start_block, prev_block))
                        start_block = None
                if start_block is not None and prev_block is not None:
                    zero_blocks.append((start_block, prev_block))

                for start, end in zero_blocks:
                    f.write(f'In #{idx} frame, block numbers {start} to {end} are zero\n')

            data = np.column_stack((scan.measurement_id, scan.timestamp))
            data[:, 1] -= 37000000000
            csv_filename = f'{pcd_path}.csv'
            np.savetxt(csv_filename, data, delimiter=',', header='measurement_id,timestamp', comments='', fmt='%d')
            pc.save(pcd_path)

            # Append TIMESTAMP line to the PCD file header
            #append_timestamp_to_pcd_header(pcd_path, scan.timestamp[0])


def pcap_read_packets(
        source: client.PacketSource,
        metadata: client.SensorInfo,
        num: int = 0  # not used in this example
) -> None:
    """Basic read packets example from pcap file. """
    # [doc-stag-pcap-read-packets]
    for packet in source:
        if isinstance(packet, client.LidarPacket):
            # Now we can process the LidarPacket. In this case, we access
            # the measurement ids, timestamps, and ranges
            measurement_ids = packet.measurement_id
            timestamps = packet.timestamp
            ranges = packet.field(client.ChanField.RANGE)
            print(f'  encoder counts = {measurement_ids.shape}')
            print(f'  timestamps = {timestamps.shape}')
            print(f'  ranges = {ranges.shape}')

        elif isinstance(packet, client.ImuPacket):
            # and access ImuPacket content
            print(f'  acceleration = {packet.accel}')
            print(f'  angular_velocity = {packet.angular_vel}')
    # [doc-etag-pcap-read-packets]


def pcap_to_csv(
        source: client.PacketSource,
        metadata: client.SensorInfo,
        num: int = 0) -> None:
    # leave comment directing users to ouster-cli
    print("NOTICE: The pcap-to-csv example has been retired in favor of "
          "the ouster-cli utility installed with the Python Ouster SDK.\n"
          "To try: ouster-cli source <PCAP> convert <OUT.CSV>")


def main():
    """Pcap examples runner."""
    examples = {
        "pcap-to-pcd": pcap_to_pcd,
        "pcap-to-csv": pcap_to_csv,
        "read-packets": pcap_read_packets,
    }

    description = "Ouster Python SDK Pcap examples. The EXAMPLE must be one of:\n  " + str.join(
        '\n  ', examples.keys())

    parser = argparse.ArgumentParser(
        description=description, formatter_class=argparse.RawTextHelpFormatter)

    parser.add_argument('pcap_path', metavar='PCAP', help='path to pcap file')
    parser.add_argument('metadata_path',
                        metavar='METADATA',
                        help='path to metadata json')
    parser.add_argument('example',
                        metavar='EXAMPLE',
                        choices=examples.keys(),
                        help='name of the example to run')
    parser.add_argument('--scan-num',
                        type=int,
                        default=0,
                        help='index of scan to use')
    parser.add_argument('--output-dir', '-o',
                    metavar='OUTPUT_DIR',
                    default="./output",
                    help='path to directory for storing PCD and CSV files')
    args = parser.parse_args()

    try:
        example = examples[args.example]
    except KeyError:
        print(f"No such example: {args.example}")
        print(description)
        exit(1)

    if not args.metadata_path or not os.path.exists(args.metadata_path):
        print(f"Metadata file does not exist: {args.metadata_path}")
        exit(1)

    with open(args.metadata_path, 'r') as f:
        metadata = client.SensorInfo(f.read())

    source = pcap.Pcap(args.pcap_path, metadata)

    with closing(source):
        example(source, metadata, args.scan_num, args.output_dir)  # type: ignore


if __name__ == "__main__":
    main()


