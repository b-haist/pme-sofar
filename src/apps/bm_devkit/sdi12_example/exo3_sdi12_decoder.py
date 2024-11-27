from bitstring import BitStream
import struct

# Test sensor-data payloads in hex format.
payload = "de00000000bb9ba012215ed80006d40000caf3e2ef0ef2e48b51b585e00c02a8410000a03fae47e940cdccf4c1f628d54200001841cdcc204114aea73f48e1ba3e85eb4141a01aa8415c8fa23f52b8e640b81ed3c11f05d5420ad71741b81e214148e19a3f48e1ba3e85eb41413333a8415c8fa23f52b8e640295cd1c13d0ad5420ad717419a99214148e19a3f48e1ba3e85eb4141"

# Description of the detection structure to unpack from the payload.
# Each tuple contains a type and a field name.
# This is a representation of the C struct the data is serialized from:
#       struct __attribute__((packed)) EXO3sample {
#               float temp_sensor;    // Celcius
#               float sp_cond;        // uS/cm
#               float pH;
#               float pH_mV;          // mV
#               float dis_oxy;        // % Sat
#               float dis_oxy_mg;     // mg/L
#               float turbidity;      // NTU
#               float wiper_pos;      // volt
#               float depth;          // meters
#               float power;          // volt
#             };
detect_struct_description = [
    ('float', 'temp_sensor'),       # Celcius
    ('float', 'sp_cond'),           # uS/cm
    ('float', 'pH'),                # range: 1-14
    ('float', 'pH_mV'),             # mV
    ('float', 'dis_oxy'),           # % Sat
    ('float', 'dis_oxy_mg'),        # mg/L
    ('float', 'turbidity'),         # NTU
    ('float', 'wiper_pos'),         # Volt
    ('float', 'depth'),             # meters
    ('float', 'power'),             # Volt
]


def hex_to_struct(hex_data, struct_description):
    """
    Converts hex data to a structured format using a provided struct description.

    Args:
        hex_data: The hexadecimal string or bytes to be converted.
        struct_description: A list of tuples, where each tuple defines the type
                            and name of each field in the struct.

    Returns:
        A dictionary where keys are field names from the struct description and
        values are the corresponding unpacked data.

    Raises:
        ValueError: If hex_data is neither a string nor bytes, or if the hex_data
                    does not match the expected struct size.
    """
    # Convert the hex data to bytes.
    if type(hex_data) is str:
        byte_data = bytes.fromhex(hex_data.strip())
    elif type(hex_data) is bytes:
        byte_data = hex_data
    else:
        raise ValueError(f'unsupported hex_data type: {type(hex_data)}')

    # Create the format string for struct.unpack based on the struct description.
    # Using little-endian ('<') for the format string.
    format_string = '<'
    for data_type, _ in struct_description:
        # Mapping of data types to format codes for struct.unpack.
        if data_type == 'uint8_t':
            format_string += 'B'  # Unsigned char (1 byte)
        elif data_type == 'uint16_t':
            format_string += 'H'  # Unsigned short (2 bytes)
        elif data_type == 'uint32_t':
            format_string += 'I'  # Unsigned int (4 bytes)
        elif data_type == 'uint64_t':
            format_string += 'Q'  # Unsigned long long (8 bytes)
        elif data_type == 'int8_t':
            format_string += 'b'  # Signed char (1 byte)
        elif data_type == 'int16_t':
            format_string += 'h'  # Signed short (2 bytes)
        elif data_type == 'int32_t':
            format_string += 'i'  # Signed int (4 bytes)
        elif data_type == 'int64_t':
            format_string += 'q'  # Signed long long (8 bytes)
        elif data_type == 'float':
            format_string += 'f'  # Float (4 bytes)
        elif data_type == 'double':
            format_string += 'd'  # Double (8 bytes)
        elif data_type == 'char':
            format_string += 'c'  # Single character
        else:
            raise ValueError(f"Unsupported data type: {data_type}")

    # Check the total size of the struct.
    expected_size = struct.calcsize(format_string)
    if len(byte_data) != expected_size:
        raise ValueError(f"Expected {expected_size} bytes, but got {len(byte_data)} bytes")

    # Unpack the data from the byte array based on the format string.
    values = struct.unpack(format_string, byte_data)

    # Convert the unpacked values into a dictionary with field names.
    result = {name: value for (_, name), value in zip(struct_description, values)}

    return result


if __name__ == '__main__':
    # Your hexadecimal data as bytes
    hex_data = bytes.fromhex(payload)

    # Load into a BitStream
    bitstream = BitStream(hex_data)

    # To read only raw data, skip the first 29 header bytes by reading and discarding them
    _ = bitstream.read('bytes:29')

    # Process remaining bits in the bitstream to extract detection data.
    while bitstream.pos < bitstream.len:
        # Read the next 11 bytes representing detection data.
        detect_data = bitstream.read('bytes:40')

        # Convert detection data from bytes to a structured format.
        detection_data = hex_to_struct(detect_data, detect_struct_description)

        # Print the unpacked detection data.
        print(f"- Detection data:")
        for key in detection_data:
            print(f"\t{key}: {float(detection_data[key]):.3f}")

        print("---------------------------------\n")