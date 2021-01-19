use crate::las_points::LasPoints;
use anyhow::Result;
use byteorder::{LittleEndian, WriteBytesExt};
use std::convert::TryInto;
use std::io::{Seek, SeekFrom, Write};

/**
 * Writes the given blob to the given writer as a single uncompressed blob. Returns the offset to the start and end of the
 * blob in the writer (end offset is exclusive)
 */
pub fn write_uncompressed_blob<W: std::io::Write + std::io::Seek>(
    blob: &[u8],
    writer: &mut W,
) -> Result<std::ops::Range<u64>> {
    let start_offset = writer.seek(SeekFrom::Current(0))?;

    writer.write(blob)?;

    let end_offset = writer.seek(SeekFrom::Current(0))?;

    Ok(start_offset..end_offset)
}

/**
 * Writes the given blob to the given writer as a single LZ4-compressed blob. Returns the offset to the start and end of the
 * compressed blob in the writer (end offset is exclusive)
 */
pub fn write_compressed_blob<W: std::io::Write + std::io::Seek>(
    blob: &[u8],
    writer: &mut W,
    compression_factor: u32,
) -> Result<std::ops::Range<u64>> {
    let start_offset = writer.seek(SeekFrom::Current(0))?;
    let mut compressed_writer = lz4::EncoderBuilder::new()
        .level(compression_factor)
        .build(writer)?;

    compressed_writer.write(blob)?;

    let (w, result) = compressed_writer.finish();
    //writer = w;
    result?;

    let end_offset = w.seek(SeekFrom::Current(0))?;

    Ok(start_offset..end_offset)
}

pub fn write_uncompressed_block<W: std::io::Write + std::io::Seek>(
    point_buffer: &LasPoints,
    writer: &mut W,
) -> Result<u64> {
    let start_index = writer.seek(SeekFrom::Current(0))?;

    // TODO My initial idea was to also store the offset to each attribute-blob in the current block at the start of the block
    // This is probably overkill, as for uncompressed LASER files these offsets will always be the same in each block (except
    // the last block, which might contain less than block_size points)
    // Both cases can be handled by a decoder through some simple calculations when opening the LASER file

    // Write all attributes as compressed blobs
    write_uncompressed_blob(point_buffer.xyz(), writer)?;

    write_uncompressed_blob(point_buffer.intensities(), writer)?;

    write_uncompressed_blob(point_buffer.bit_attributes(), writer)?;

    write_uncompressed_blob(point_buffer.classifications(), writer)?;

    write_uncompressed_blob(point_buffer.user_data(), writer)?;
    write_uncompressed_blob(point_buffer.scan_angle_ranks(), writer)?;

    write_uncompressed_blob(point_buffer.source_ids(), writer)?;

    match point_buffer.gps_times() {
        Some(gps) => {
            write_uncompressed_blob(gps, writer)?;
        }
        _ => (),
    }

    match point_buffer.rgbs() {
        Some(rgbs) => {
            write_uncompressed_blob(rgbs, writer)?;
        }
        _ => (),
    }

    match point_buffer.nirs() {
        Some(nirs) => {
            write_uncompressed_blob(nirs, writer)?;
        }
        _ => (),
    }

    match point_buffer.waveforms() {
        Some(waveforms) => {
            write_uncompressed_blob(waveforms, writer)?;
        }
        _ => (),
    }

    write_uncompressed_blob(point_buffer.extra_bytes(), writer)?;

    Ok(start_index)
}

pub fn write_compressed_block<W: std::io::Write + std::io::Seek>(
    point_buffer: &LasPoints,
    writer: &mut W,
    compression_factor: u32,
) -> Result<u64> {
    let start_index = writer.seek(SeekFrom::Current(0))?;

    let number_of_attributes = point_buffer.number_of_attributes();
    let mut attribute_offsets: Vec<u64> = Vec::with_capacity(
        number_of_attributes
            .try_into()
            .expect("Number of point attributes exceeds usize::MAX on current platform"),
    );

    // Memorize the location where we will write the offset to the attribute blobs into
    let offset_to_file_attributes_offset_table = writer.seek(SeekFrom::Current(0))?;
    // Reserve the necessary space for the attribute offsets. Each offset is a single u64 value
    writer.seek(SeekFrom::Current(number_of_attributes as i64 * 8))?;

    // Write all attributes as compressed blobs
    let attrib_xyz_range = write_compressed_blob(point_buffer.xyz(), writer, compression_factor)?;
    attribute_offsets.push(attrib_xyz_range.start);

    let attrib_intensity_range =
        write_compressed_blob(point_buffer.intensities(), writer, compression_factor)?;
    attribute_offsets.push(attrib_intensity_range.start);

    let attrib_bit_attributes_range =
        write_compressed_blob(point_buffer.bit_attributes(), writer, compression_factor)?;
    attribute_offsets.push(attrib_bit_attributes_range.start);

    let attrib_classifications_range =
        write_compressed_blob(point_buffer.classifications(), writer, compression_factor)?;
    attribute_offsets.push(attrib_classifications_range.start);

    let attrib_scan_angle_ranks_range =
        write_compressed_blob(point_buffer.scan_angle_ranks(), writer, compression_factor)?;
    attribute_offsets.push(attrib_scan_angle_ranks_range.start);

    let attrib_user_data_range =
        write_compressed_blob(point_buffer.user_data(), writer, compression_factor)?;
    attribute_offsets.push(attrib_user_data_range.start);

    let attrib_source_ids_range =
        write_compressed_blob(point_buffer.source_ids(), writer, compression_factor)?;
    attribute_offsets.push(attrib_source_ids_range.start);

    let attrib_extra_bytes_range =
        write_compressed_blob(point_buffer.extra_bytes(), writer, compression_factor)?;
    attribute_offsets.push(attrib_extra_bytes_range.start);

    let mut end_of_block = attrib_extra_bytes_range.end;

    match point_buffer.rgbs() {
        Some(rgbs) => {
            let attrib_rgb_range = write_compressed_blob(rgbs, writer, compression_factor)?;
            attribute_offsets.push(attrib_rgb_range.start);
            end_of_block = attrib_rgb_range.end;
        }
        _ => (),
    }

    match point_buffer.gps_times() {
        Some(gps) => {
            let attrib_gps_range = write_compressed_blob(gps, writer, compression_factor)?;
            attribute_offsets.push(attrib_gps_range.start);
            end_of_block = attrib_gps_range.end;
        }
        _ => (),
    }

    match point_buffer.waveforms() {
        Some(waveforms) => {
            let attrib_waveform_range =
                write_compressed_blob(waveforms, writer, compression_factor)?;
            attribute_offsets.push(attrib_waveform_range.start);
            end_of_block = attrib_waveform_range.end;
        }
        _ => (),
    }

    match point_buffer.nirs() {
        Some(nirs) => {
            let attrib_nir_range = write_compressed_blob(nirs, writer, compression_factor)?;
            attribute_offsets.push(attrib_nir_range.start);
            end_of_block = attrib_nir_range.end;
        }
        _ => (),
    }

    // Write the actual attribute offsets
    writer.seek(SeekFrom::Start(offset_to_file_attributes_offset_table))?;
    for offset in attribute_offsets.into_iter() {
        writer.write_u64::<LittleEndian>(offset)?;
    }

    // Make sure we move back to the end of the block so that the next block is written at the correct position
    writer.seek(SeekFrom::Start(end_of_block))?;

    Ok(start_index)
}
