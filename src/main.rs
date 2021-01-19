use crate::writer::write_compressed_blob;
use crate::writer::write_compressed_block;
use crate::writer::write_uncompressed_block;
use las::Read;
use rayon::prelude::*;
use std::convert::TryInto;
use {
    anyhow::{anyhow, Context, Result},
    byteorder::{LittleEndian, WriteBytesExt},
    clap::{load_yaml, value_t, App, Arg},
    las::{Point, Reader},
    std::fs::{copy, read_dir, File},
    std::io::{BufReader, BufWriter, Seek, SeekFrom, Write},
    std::path::{Path, PathBuf},
    std::str::FromStr,
    std::time::{Duration, Instant},
};

mod las_points;
pub mod writer;

#[derive(Debug, PartialEq, Copy, Clone)]
enum OutputFormat {
    LAST,
    LAZT,
    LASER,
    LAZER,
}

impl FromStr for OutputFormat {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        match s {
            "LAST" => Ok(OutputFormat::LAST),
            "LAZT" => Ok(OutputFormat::LAZT),
            "LASER" => Ok(OutputFormat::LASER),
            "LAZER" => Ok(OutputFormat::LAZER),
            _ => Err(anyhow!("Could not parse string \"{}\" as OutputFormat!", s)),
        }
    }
}

const LAS_V1_3_HEADER_SIZE_BYTES: usize = 227;
const LAS_V1_4_HEADER_SIZE_BYTES: usize = 375;

fn get_header<R: std::io::Read>(reader: &mut R) -> Result<las::raw::Header> {
    let header = las::raw::Header::read_from(reader)?;
    Ok(header)
}

fn copy_header<R: std::io::Read, W: std::io::Write>(reader: &mut R, writer: &mut W) -> Result<()> {
    let mut buffer = vec![0; LAS_V1_4_HEADER_SIZE_BYTES];
    let bytes_read = reader.read(&mut buffer)?;
    if bytes_read < LAS_V1_3_HEADER_SIZE_BYTES {
        return Err(anyhow!("Could not read LAS header"));
    }

    //Bytes 94 and 95 correspond to the header size
    let header_size = u16::from_le_bytes([buffer[94], buffer[95]]) as usize;
    writer.write(&buffer[0..header_size])?;

    Ok(())
}

fn copy_vlrs<R: std::io::Read, W: std::io::Write>(
    reader: &mut R,
    writer: &mut W,
    size_of_vlrs: u64,
) -> Result<()> {
    // Copy all VLRs from reader to writer. reader is assumed to be at the first byte of the first VLR
    let mut buffer: Vec<u8> = Vec::new();
    buffer.resize(
        size_of_vlrs
            .try_into()
            .expect("Could not convert size of VLR section to usize"),
        0,
    );
    reader.read_exact(buffer.as_mut_slice())?;
    writer.write_all(buffer.as_mut_slice())?;

    Ok(())
}

fn format_bytes(bytes: usize) -> String {
    let mut bytes_as_f64 = bytes as f64;
    let units = ["B", "KiB", "MiB", "GiB", "TiB", "PiB", "EiB"];
    let mut unit_index = 0;
    while bytes_as_f64 >= 1024.0 && unit_index < units.len() - 1 {
        bytes_as_f64 /= 1024.0;
        unit_index += 1;
    }
    if unit_index == 0 {
        format!("{} {}", bytes, units[0])
    } else {
        format!("{:.3} {}", bytes_as_f64, units[unit_index])
    }
}

fn log_attribute_sizes(point_buffer: &las_points::LasPoints) {
    println!("Point cloud attribute sizes:");
    println!(
        "Positions:        {}",
        format_bytes(point_buffer.xyz().len())
    );
    println!(
        "Intensities:      {}",
        format_bytes(point_buffer.intensities().len())
    );
    println!(
        "Bit flags:        {}",
        format_bytes(point_buffer.bit_attributes().len())
    );
    println!(
        "Classifications:  {}",
        format_bytes(point_buffer.classifications().len())
    );
    println!(
        "Scan angle ranks: {}",
        format_bytes(point_buffer.scan_angle_ranks().len())
    );
    println!(
        "User data:        {}",
        format_bytes(point_buffer.user_data().len())
    );
    println!(
        "Source IDs:       {}",
        format_bytes(point_buffer.source_ids().len())
    );
    println!(
        "Extra bytes:      {}",
        format_bytes(point_buffer.extra_bytes().len())
    );

    println!(
        "RGB colors:       {}",
        format_bytes(point_buffer.rgbs().map(|v| v.len()).unwrap_or(0))
    );
    println!(
        "GPS times:        {}",
        format_bytes(point_buffer.gps_times().map(|v| v.len()).unwrap_or(0))
    );
    println!(
        "Waveforms:        {}",
        format_bytes(point_buffer.waveforms().map(|v| v.len()).unwrap_or(0))
    );
    println!(
        "NIRs:             {}",
        format_bytes(point_buffer.nirs().map(|v| v.len()).unwrap_or(0))
    );
}

// TODOs
// *  Read file sequentially into memory and write in chunks of N points (so that we can convert files that are larger than the amount of available RAM)
// *  LasPoints structure could store a HashMap<Attribute, Vec<u8>>, would make the code easier to read

fn las_to_last(in_path: &Path, out_path: &Path, verbose: bool) -> Result<()> {
    let mut writer = BufWriter::new(File::create(out_path)?);
    let mut in_file = BufReader::new(File::open(in_path)?);
    copy_header(&mut in_file, &mut writer)?;

    in_file.seek(SeekFrom::Start(0))?;
    let raw_header = get_header(&mut in_file)?;
    let size_of_vlrs = raw_header.offset_to_point_data - raw_header.header_size as u32;
    copy_vlrs(&mut in_file, &mut writer, size_of_vlrs as u64)?;

    let current_writer_pos = writer.seek(SeekFrom::Current(0))?;
    assert!(current_writer_pos == raw_header.offset_to_point_data as u64);

    in_file.seek(SeekFrom::Start(0))?;
    let mut reader = Reader::new(in_file)?;

    let las_header = reader.header();
    if verbose {
        println!("File header:\n{:#?}", las_header);
    }

    let mut point_buffer = las_points::LasPoints::from_format(
        las_header.point_format(),
        las_header
            .number_of_points()
            .try_into()
            .expect("Could not convert number of points to usize!"),
    )?;

    let mut t_start = Instant::now();
    let transforms = reader.header().transforms().clone();

    for (idx, maybe_point) in reader.points().enumerate() {
        if idx > 0 && idx % 1000000 == 0 {
            let delta_t = t_start.elapsed();
            t_start = Instant::now();
            let pps = 1.0 / delta_t.as_secs_f64();
            println!("Read {} M points [{:.2} Mpts/s]", idx, pps);
        }

        let point = maybe_point?;
        point_buffer.add_point(point, &transforms)?;
    }

    if verbose {
        log_attribute_sizes(&point_buffer);
    }

    writer.write(point_buffer.xyz())?;
    writer.write(point_buffer.intensities())?;
    writer.write(point_buffer.bit_attributes())?;
    writer.write(point_buffer.classifications())?;
    writer.write(point_buffer.user_data())?;
    writer.write(point_buffer.scan_angle_ranks())?;
    writer.write(point_buffer.source_ids())?;
    match point_buffer.gps_times() {
        Some(gps) => {
            writer.write(gps)?;
        }
        _ => (),
    }

    match point_buffer.rgbs() {
        Some(rgbs) => {
            writer.write(rgbs)?;
        }
        _ => (),
    }

    match point_buffer.nirs() {
        Some(nirs) => {
            writer.write(nirs)?;
        }
        _ => (),
    }

    match point_buffer.waveforms() {
        Some(waveforms) => {
            writer.write(waveforms)?;
        }
        _ => (),
    }

    writer.write(point_buffer.extra_bytes())?;

    Ok(())
}

fn las_to_lazt(in_path: &Path, out_path: &Path, compression: u32, verbose: bool) -> Result<()> {
    let mut uncompressed_writer = BufWriter::new(File::create(out_path)?);
    let mut in_file = BufReader::new(File::open(in_path)?);
    copy_header(&mut in_file, &mut uncompressed_writer)?;

    in_file.seek(SeekFrom::Start(0))?;
    let raw_header = get_header(&mut in_file)?;
    let size_of_vlrs = raw_header.offset_to_point_data - raw_header.header_size as u32;
    copy_vlrs(&mut in_file, &mut uncompressed_writer, size_of_vlrs as u64)?;

    in_file.seek(SeekFrom::Start(0))?;
    let mut reader = Reader::new(in_file)?;

    let las_header = reader.header();
    if verbose {
        println!("File header:\n{:#?}", las_header);
    }

    let mut point_buffer = las_points::LasPoints::from_format(
        las_header.point_format(),
        las_header
            .number_of_points()
            .try_into()
            .expect("Could not convert number of points to usize!"),
    )?;

    let mut t_start = Instant::now();
    let transforms = reader.header().transforms().clone();

    for (idx, maybe_point) in reader.points().enumerate() {
        if idx > 0 && idx % 1000000 == 0 {
            let delta_t = t_start.elapsed();
            t_start = Instant::now();
            let pps = 1.0 / delta_t.as_secs_f64();
            println!("Read {} M points [{:.2} Mpts/s]", idx, pps);
        }

        let point = maybe_point?;
        point_buffer.add_point(point, &transforms)?;
    }

    if verbose {
        log_attribute_sizes(&point_buffer);
    }

    let number_of_attributes = point_buffer.number_of_attributes();
    let mut attribute_offsets: Vec<u64> = Vec::with_capacity(
        number_of_attributes
            .try_into()
            .expect("Number of point attributes exceeds usize::MAX on current platform"),
    );

    // Memorize the location where we will write the offset to the attribute blobs into
    let offset_to_file_attributes_offset_table = uncompressed_writer.seek(SeekFrom::Current(0))?;
    // Reserve the necessary space for the attribute offsets. Each offset is a single u64 value
    uncompressed_writer.seek(SeekFrom::Current(number_of_attributes.into()))?;

    // Write all attributes as compressed blobs
    let attrib_xyz_range =
        write_compressed_blob(point_buffer.xyz(), &mut uncompressed_writer, compression)?;
    attribute_offsets.push(attrib_xyz_range.start);

    let attrib_intensity_range = write_compressed_blob(
        point_buffer.intensities(),
        &mut uncompressed_writer,
        compression,
    )?;
    attribute_offsets.push(attrib_intensity_range.start);

    let attrib_bit_attributes_range = write_compressed_blob(
        point_buffer.bit_attributes(),
        &mut uncompressed_writer,
        compression,
    )?;
    attribute_offsets.push(attrib_bit_attributes_range.start);

    let attrib_classifications_range = write_compressed_blob(
        point_buffer.classifications(),
        &mut uncompressed_writer,
        compression,
    )?;
    attribute_offsets.push(attrib_classifications_range.start);

    let attrib_scan_angle_ranks_range = write_compressed_blob(
        point_buffer.scan_angle_ranks(),
        &mut uncompressed_writer,
        compression,
    )?;
    attribute_offsets.push(attrib_scan_angle_ranks_range.start);

    let attrib_user_data_range = write_compressed_blob(
        point_buffer.user_data(),
        &mut uncompressed_writer,
        compression,
    )?;
    attribute_offsets.push(attrib_user_data_range.start);

    let attrib_source_ids_range = write_compressed_blob(
        point_buffer.source_ids(),
        &mut uncompressed_writer,
        compression,
    )?;
    attribute_offsets.push(attrib_source_ids_range.start);

    let attrib_extra_bytes_range = write_compressed_blob(
        point_buffer.extra_bytes(),
        &mut uncompressed_writer,
        compression,
    )?;
    attribute_offsets.push(attrib_extra_bytes_range.start);

    match point_buffer.gps_times() {
        Some(gps) => {
            let attrib_gps_range =
                write_compressed_blob(gps, &mut uncompressed_writer, compression)?;
            attribute_offsets.push(attrib_gps_range.start);
        }
        _ => (),
    }

    match point_buffer.rgbs() {
        Some(rgbs) => {
            let attrib_rgb_range =
                write_compressed_blob(rgbs, &mut uncompressed_writer, compression)?;
            attribute_offsets.push(attrib_rgb_range.start);
        }
        _ => (),
    }

    match point_buffer.nirs() {
        Some(nirs) => {
            let attrib_nir_range =
                write_compressed_blob(nirs, &mut uncompressed_writer, compression)?;
            attribute_offsets.push(attrib_nir_range.start);
        }
        _ => (),
    }

    match point_buffer.waveforms() {
        Some(waveforms) => {
            let attrib_waveform_range =
                write_compressed_blob(waveforms, &mut uncompressed_writer, compression)?;
            attribute_offsets.push(attrib_waveform_range.start);
        }
        _ => (),
    }

    if verbose {
        println!("Attribute offsets:");
        for offset in attribute_offsets.iter() {
            println!("{}", offset);
        }
    }

    // Write the actual attribute offsets
    uncompressed_writer.seek(SeekFrom::Start(offset_to_file_attributes_offset_table))?;
    for offset in attribute_offsets.into_iter() {
        uncompressed_writer.write_u64::<LittleEndian>(offset)?;
    }

    Ok(())
}

fn las_to_laser(in_path: &Path, out_path: &Path, block_size: u32, verbose: bool) -> Result<()> {
    const MIN_BLOCK_SIZE: u32 = 1024;
    if block_size < MIN_BLOCK_SIZE {
        return Err(anyhow!(
            "Blocksize {} is less than minimum allowed blocksize {}",
            block_size,
            MIN_BLOCK_SIZE
        ));
    }
    let blocksize_usize: usize = block_size
        .try_into()
        .expect("Could not convert blocksize from u32 to usize");

    let mut uncompressed_writer = BufWriter::new(File::create(out_path)?);
    let mut in_file = BufReader::new(File::open(in_path)?);
    copy_header(&mut in_file, &mut uncompressed_writer)?;

    in_file.seek(SeekFrom::Start(0))?;
    let raw_header = get_header(&mut in_file)?;
    let size_of_vlrs = raw_header.offset_to_point_data - raw_header.header_size as u32;
    copy_vlrs(&mut in_file, &mut uncompressed_writer, size_of_vlrs as u64)?;

    in_file.seek(SeekFrom::Start(0))?;
    let mut reader = Reader::new(in_file)?;

    let las_header = reader.header();
    if verbose {
        println!("File header:\n{:#?}", las_header);
    }

    uncompressed_writer.write_u64::<LittleEndian>(block_size as u64)?;

    let number_of_blocks =
        f64::ceil((las_header.number_of_points() as f64) / (block_size as f64)) as u64;

    let size_of_block_offsets = number_of_blocks * 8;
    // Memorize the offset in the file where we will write the block offsets
    let offset_to_block_offsets = uncompressed_writer.seek(SeekFrom::Current(0))?;
    // Reserve memory for the block offsets
    uncompressed_writer.seek(SeekFrom::Current(
        size_of_block_offsets
            .try_into()
            .expect("Size of block offsets is too large for usize"),
    ))?;

    let mut point_buffer =
        las_points::LasPoints::from_format(las_header.point_format(), blocksize_usize)?;
    let mut block_offsets: Vec<u64> = Vec::with_capacity(number_of_blocks as usize);

    let transforms = reader.header().transforms().clone();

    for (idx, point_or_err) in reader.points().enumerate() {
        if idx > 0 && idx % blocksize_usize == 0 {
            let block_offset = write_uncompressed_block(&point_buffer, &mut uncompressed_writer)?;
            println!(
                "Writing LASER block {} at offset {}",
                idx / blocksize_usize,
                block_offset
            );
            block_offsets.push(block_offset);
            point_buffer.clear();
        }

        let point = point_or_err?;
        point_buffer.add_point(point, &transforms)?;
    }

    if point_buffer.point_count() > 0 {
        let block_offset = write_uncompressed_block(&point_buffer, &mut uncompressed_writer)?;
        block_offsets.push(block_offset);
    }

    // Write block offsets
    uncompressed_writer.seek(SeekFrom::Start(offset_to_block_offsets))?;
    for block_offset in block_offsets.into_iter() {
        uncompressed_writer.write_u64::<LittleEndian>(block_offset)?;
    }

    Ok(())
}

fn las_to_lazer(
    in_path: &Path,
    out_path: &Path,
    block_size: u32,
    compression: u32,
    verbose: bool,
) -> Result<()> {
    //TODO Make sure the offset fields in the header are correct! Especially for the EVLRs

    const MIN_BLOCK_SIZE: u32 = 1024;
    if block_size < MIN_BLOCK_SIZE {
        return Err(anyhow!(
            "Blocksize {} is less than minimum allowed blocksize {}",
            block_size,
            MIN_BLOCK_SIZE
        ));
    }
    let blocksize_usize: usize = block_size
        .try_into()
        .expect("Could not convert blocksize from u32 to usize");

    let mut uncompressed_writer = BufWriter::new(File::create(out_path)?);
    let mut in_file = BufReader::new(File::open(in_path)?);
    copy_header(&mut in_file, &mut uncompressed_writer)?;

    in_file.seek(SeekFrom::Start(0))?;
    let raw_header = get_header(&mut in_file)?;
    let size_of_vlrs = raw_header.offset_to_point_data - raw_header.header_size as u32;
    copy_vlrs(&mut in_file, &mut uncompressed_writer, size_of_vlrs as u64)?;

    in_file.seek(SeekFrom::Start(0))?;
    let mut reader = Reader::new(in_file)?;

    let las_header = reader.header();
    if verbose {
        println!("File header:\n{:#?}", las_header);
    }

    uncompressed_writer.write_u64::<LittleEndian>(block_size as u64)?;

    let number_of_blocks =
        f64::ceil((las_header.number_of_points() as f64) / (block_size as f64)) as usize;
    let size_of_block_offsets = number_of_blocks * 8;
    // Memorize the offset in the file where we will write the block offsets
    let offset_to_block_offsets = uncompressed_writer.seek(SeekFrom::Current(0))?;
    // Reserve memory for the block offsets
    uncompressed_writer.seek(SeekFrom::Current(
        size_of_block_offsets
            .try_into()
            .expect("Size of block offsets is too large for usize"),
    ))?;

    let mut point_buffer =
        las_points::LasPoints::from_format(las_header.point_format(), blocksize_usize)?;
    let mut block_offsets: Vec<u64> = Vec::with_capacity(number_of_blocks);

    let transforms = reader.header().transforms().clone();

    for (idx, point_or_err) in reader.points().enumerate() {
        if idx > 0 && idx % blocksize_usize == 0 {
            if verbose {
                println!("Writing LAZER block {}", idx / blocksize_usize);
            }
            let block_offset =
                write_compressed_block(&point_buffer, &mut uncompressed_writer, compression)?;
            block_offsets.push(block_offset);
            point_buffer.clear();
        }

        let point = point_or_err?;
        point_buffer.add_point(point, &transforms)?;
    }

    if point_buffer.point_count() > 0 {
        let block_offset =
            write_compressed_block(&point_buffer, &mut uncompressed_writer, compression)?;
        block_offsets.push(block_offset);
    }

    // Write block offsets
    uncompressed_writer.seek(SeekFrom::Start(offset_to_block_offsets))?;
    for block_offset in block_offsets.into_iter() {
        uncompressed_writer.write_u64::<LittleEndian>(block_offset)?;
    }

    Ok(())
}

fn convert_file(
    in_path: &Path,
    out_path: &Path,
    output_format: &OutputFormat,
    block_size: Option<u32>,
    compression: Option<u32>,
    verbose: bool,
) -> Result<()> {
    println!(
        "Converting file {} to {}",
        in_path.display(),
        out_path.display()
    );
    match output_format {
        OutputFormat::LASER => las_to_laser(
            in_path,
            out_path,
            block_size.expect("Argument BLOCKSIZE is required for LAS to LASER conversion"),
            verbose,
        ),
        OutputFormat::LAST => las_to_last(in_path, out_path, verbose),
        OutputFormat::LAZER => las_to_lazer(
            in_path,
            out_path,
            block_size.expect("Argument BLOCKSIZE is required for LAS to LAZER conversion"),
            compression.expect("Argument COMPRESSION is required for LAS to LAZER conversion"),
            verbose,
        ),
        OutputFormat::LAZT => las_to_lazt(
            in_path,
            out_path,
            compression.expect("Argument COMPRESSION is required for LAS to LAZER conversion"),
            verbose,
        ),
    }
}

fn is_valid_file(in_path: &Path) -> bool {
    match in_path.extension() {
        None => false,
        Some(ex) => ex == "las" || ex == "laz",
    }
}

fn get_input_files(in_path: &Path) -> Result<Vec<PathBuf>> {
    if in_path.is_file() {
        Ok(vec![in_path.into()])
    } else if in_path.is_dir() {
        read_dir(in_path)?
            .map(|res| res.map(|e| e.path()))
            .filter(|res| res.is_ok() && is_valid_file(res.as_ref().unwrap()))
            .collect::<Result<Vec<_>, std::io::Error>>()
            .context("")
    } else {
        Err(anyhow!(
            "Input path {} is neither file nor directory!",
            in_path.display()
        ))
    }
}

fn get_output_file_from_input_file_name(
    in_path: &Path,
    out_path: &Path,
    output_format: OutputFormat,
) -> Result<PathBuf> {
    let file_name = in_path.file_stem().ok_or(anyhow!(
        "Could not get file stem from path \"{}\"",
        in_path.display()
    ))?;
    let mut file_name_with_new_extension = file_name.to_owned();
    match output_format {
        OutputFormat::LASER => file_name_with_new_extension.push(".laser"),
        OutputFormat::LAST => file_name_with_new_extension.push(".last"),
        OutputFormat::LAZER => file_name_with_new_extension.push(".lazer"),
        OutputFormat::LAZT => file_name_with_new_extension.push(".lazt"),
    };
    Ok(out_path.join(file_name_with_new_extension))
}

fn main() -> Result<()> {
    let yaml = load_yaml!("cli.yaml");
    let matches = App::from_yaml(&yaml).get_matches();

    let in_path = Path::new(
        matches
            .value_of("INPUT")
            .expect("Argument INPUT is missing!"),
    );
    let out_path = Path::new(
        matches
            .value_of("OUTPUT")
            .expect("Argument OUTPUT is missing!"),
    );
    let output_format = OutputFormat::from_str(
        matches
            .value_of("FORMAT")
            .expect("Argument FORMAT is missing!"),
    )?;
    let maybe_block_size = match matches.value_of("blocksize") {
        None => None,
        Some(s) => Some(u32::from_str(s)?),
    };
    let maybe_compression = match matches.value_of("compression") {
        None => None,
        Some(s) => Some(u32::from_str(s)?),
    };
    let is_verbose = matches.occurrences_of("verbose") > 0;

    let input_files = get_input_files(in_path)?;
    let output_files = input_files
        .iter()
        .map(|f| get_output_file_from_input_file_name(f, out_path, output_format))
        .collect::<Result<Vec<_>>>()?;

    let in_out_files = input_files
        .iter()
        .zip(output_files.iter())
        .collect::<Vec<_>>();

    if input_files.len() == 1 {
        println!("Converting 1 file...");
    } else {
        println!("Converting {} files...", input_files.len());
    }

    in_out_files
        .par_iter()
        .map(|(in_file, out_file)| {
            convert_file(
                in_file,
                out_file,
                &output_format,
                maybe_block_size,
                maybe_compression,
                is_verbose,
            )
        })
        .collect::<Result<Vec<_>>>()?;

    println!("Done!");

    Ok(())
}
