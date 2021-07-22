# Pointcloud file format conversions

A tool to convert point cloud files in the `LAS`/`LAZ` formats to other custom formats. These formats are experimental and were developed for the "Executing ad-hoc queries on large geospatial data sets without acceleration structures" paper. Besides converting from `LAS` to `LAZ` and vice versa, the following new file formats are introduced by this tool:

*  `LAST` ("LAS Transposed") is identical to the `LAS` format, but the memory layout is transposed so that all values of a single attribute are stored together in memory, as opposed to `LAS` which stores all attributes of a single _point_ together in memory
*  `LAZT` ("LAST but Zipped") is identical to `LAST`, but the whole file is compressed using the [LZ4](https://github.com/lz4/lz4) compression algorithm
*  `LAZER` ("LAZ for Efficient Reading"), a variant of `LAZ` that uses a different compression algorithm ([LZ4](https://github.com/lz4/lz4)). This format stores points in blocks of a fixed number of points, where each block has the same memory layout as the `LAST` file, preceeded by a block header. Within a block, all attributes use a separate compression context, the block header stores the offset from the start of the block to the start of the compressed attributes to enable faster attribute lookup
*  `LASER`, which is like `LAZER` but without compression

## Usage

Requires a recent Rust installation (>=1.51), or Docker. With Rust, simply run `cargo run --release` to get information about the usage. In its simplest form, converting from one format to another works like this:

```Rust
cargo run --release -- LAST source_file.las target_file.last
```

For the compressed formats, there are also some additional flags available:
*  `-c X` with `X` in [1;9] specifies the amount of compression. See the LZ4 specification for more details
*  `-b X` specifies the block size `X` (i.e. the number of points per block) for the `LASER` and `LAZER` formats. A good default is 50_000, which is the same value that `LAZ` uses 

This project is licensed under the MIT license. 