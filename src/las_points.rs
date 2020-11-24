use {
    anyhow::{anyhow, Result},
    las::point::{Format, Point},
    las::Transform,
    las::Vector,
    std::convert::TryInto,
};

fn push_bytes<T>(val: &T, vec: &mut Vec<u8>) {
    let ptr: *const T = val;
    let byte_ptr = ptr as *const u8;
    for idx in 0..std::mem::size_of::<T>() {
        unsafe {
            vec.push(*byte_ptr.offset(idx.try_into().expect("Could not convert index to isize")));
        }
    }
}

pub struct LasPoints {
    xyz: Vec<u8>,
    intensities: Vec<u8>,
    bit_attributes: Vec<u8>,
    classifications: Vec<u8>,
    scan_angle_ranks: Vec<u8>,
    user_data: Vec<u8>,
    source_ids: Vec<u8>,
    rgbs: Option<Vec<u8>>,
    gps_times: Option<Vec<u8>>,
    waveforms: Option<Vec<u8>>,
    nirs: Option<Vec<u8>>,
    extra_bytes: Vec<u8>,
    is_extended_format: bool,
}

impl LasPoints {
    pub fn from_format(format: &Format, capacity: usize) -> Result<Self> {
        let scan_angle_ranks_size = match format.to_u8().unwrap() {
            0..=5 => 1,
            6..=10 => 2,
            _ => return Err(anyhow!("Invalid LAS format {}", format)),
        };
        let bit_attributes_size = match format.to_u8().unwrap() {
            0..=5 => 1,
            6..=10 => 2,
            _ => return Err(anyhow!("Invalid LAS format {}", format)),
        };

        Ok(Self {
            xyz: Vec::with_capacity(capacity * 12),
            intensities: Vec::with_capacity(capacity * 2),
            bit_attributes: Vec::with_capacity(capacity * bit_attributes_size),
            classifications: Vec::with_capacity(capacity),
            scan_angle_ranks: Vec::with_capacity(capacity * scan_angle_ranks_size),
            user_data: Vec::with_capacity(capacity),
            source_ids: Vec::with_capacity(capacity * 2),
            rgbs: if format.has_color {
                Some(Vec::with_capacity(capacity * 6))
            } else {
                None
            },
            gps_times: if format.has_gps_time {
                Some(Vec::with_capacity(capacity * 8))
            } else {
                None
            },
            waveforms: if format.has_waveform {
                Some(Vec::with_capacity(capacity * 29))
            } else {
                None
            },
            nirs: if format.has_nir {
                Some(Vec::with_capacity(capacity * 2))
            } else {
                None
            },
            extra_bytes: Vec::with_capacity(capacity * format.extra_bytes as usize),
            is_extended_format: format.is_extended,
        })
    }

    pub fn add_point(&mut self, point: Point, transforms: &Vector<Transform>) -> Result<()> {
        let mut raw_point = point.into_raw(transforms)?;

        let x_bytes = raw_point.x.to_le_bytes();
        let y_bytes = raw_point.y.to_le_bytes();
        let z_bytes = raw_point.z.to_le_bytes();

        self.xyz.push(x_bytes[0]);
        self.xyz.push(x_bytes[1]);
        self.xyz.push(x_bytes[2]);
        self.xyz.push(x_bytes[3]);
        self.xyz.push(y_bytes[0]);
        self.xyz.push(y_bytes[1]);
        self.xyz.push(y_bytes[2]);
        self.xyz.push(y_bytes[3]);
        self.xyz.push(z_bytes[0]);
        self.xyz.push(z_bytes[1]);
        self.xyz.push(z_bytes[2]);
        self.xyz.push(z_bytes[3]);

        let intensity_bytes = raw_point.intensity.to_le_bytes();
        self.intensities.push(intensity_bytes[0]);
        self.intensities.push(intensity_bytes[1]);

        match raw_point.flags {
            las::raw::point::Flags::TwoByte(bit_flags, classification) => {
                self.bit_attributes.push(bit_flags);
                self.classifications.push(classification);
            }
            las::raw::point::Flags::ThreeByte(bit_flags_low, bit_flags_high, classification) => {
                // TODO Same as with ScanAngleRank, this is a bug in las-rs library. Flags::ThreeByte is always used when going
                // from Point to RawPoint...
                if self.is_extended_format {
                    self.bit_attributes.push(bit_flags_low);
                    self.bit_attributes.push(bit_flags_high);
                    self.classifications.push(classification);
                } else {
                    let as_two_byte_flags = raw_point.flags.to_two_bytes()?;
                    self.bit_attributes.push(as_two_byte_flags.0);
                    self.classifications.push(as_two_byte_flags.1);
                }
            }
        }
        match raw_point.scan_angle {
            las::raw::point::ScanAngle::Rank(rank) => self.scan_angle_ranks.push(rank as u8),
            las::raw::point::ScanAngle::Scaled(scaled_rank) => {
                // TODO This is a bug in the las-rs library, information about the size of the ScanAngleRank attribute (1 byte or 2 bytes)
                // is lost in conversion from Point to RawPoint...
                if self.is_extended_format {
                    let rank_bytes = scaled_rank.to_le_bytes();
                    self.scan_angle_ranks.push(rank_bytes[0]);
                    self.scan_angle_ranks.push(rank_bytes[1]);
                } else {
                    let rank_byte = scaled_rank as u8;
                    self.scan_angle_ranks.push(rank_byte);
                }
            }
        }

        self.user_data.push(raw_point.user_data);
        let source_id_bytes = raw_point.point_source_id.to_le_bytes();
        self.source_ids.push(source_id_bytes[0]);
        self.source_ids.push(source_id_bytes[1]);

        match raw_point.color {
            Some(color) => {
                let r_bytes = color.red.to_le_bytes();
                let g_bytes = color.green.to_le_bytes();
                let b_bytes = color.blue.to_le_bytes();
                let self_rgbs = self.rgbs.as_mut().unwrap();
                self_rgbs.push(r_bytes[0]);
                self_rgbs.push(r_bytes[1]);
                self_rgbs.push(g_bytes[0]);
                self_rgbs.push(g_bytes[1]);
                self_rgbs.push(b_bytes[0]);
                self_rgbs.push(b_bytes[1]);
            }
            _ => (),
        }

        match raw_point.gps_time {
            Some(gps_time) => {
                let gps_time_bytes = gps_time.to_le_bytes();
                let self_gps = self.gps_times.as_mut().unwrap();
                self_gps.push(gps_time_bytes[0]);
                self_gps.push(gps_time_bytes[1]);
                self_gps.push(gps_time_bytes[2]);
                self_gps.push(gps_time_bytes[3]);
                self_gps.push(gps_time_bytes[4]);
                self_gps.push(gps_time_bytes[5]);
                self_gps.push(gps_time_bytes[6]);
                self_gps.push(gps_time_bytes[7]);
            }
            _ => (),
        }

        match raw_point.waveform {
            Some(waveform) => {
                let self_waveform = self.waveforms.as_mut().unwrap();
                push_bytes(&waveform.wave_packet_descriptor_index, self_waveform);
                push_bytes(&waveform.byte_offset_to_waveform_data, self_waveform);
                push_bytes(&waveform.waveform_packet_size_in_bytes, self_waveform);
                push_bytes(&waveform.return_point_waveform_location, self_waveform);
                push_bytes(&waveform.x_t, self_waveform);
                push_bytes(&waveform.y_t, self_waveform);
                push_bytes(&waveform.z_t, self_waveform);
            }
            _ => (),
        }

        match raw_point.nir {
            Some(nir) => {
                let self_nir = self.nirs.as_mut().unwrap();
                push_bytes(&nir, self_nir);
            }
            _ => (),
        }

        self.extra_bytes.append(&mut raw_point.extra_bytes);

        Ok(())
    }

    pub fn clear(&mut self) {
        self.xyz.clear();
        self.intensities.clear();
        self.bit_attributes.clear();
        self.classifications.clear();
        self.scan_angle_ranks.clear();
        self.user_data.clear();
        self.source_ids.clear();
        self.extra_bytes.clear();

        match self.rgbs.as_mut() {
            Some(rgbs) => rgbs.clear(),
            _ => (),
        }

        match self.gps_times.as_mut() {
            Some(gps) => gps.clear(),
            _ => (),
        }

        match self.waveforms.as_mut() {
            Some(waveforms) => waveforms.clear(),
            _ => (),
        }

        match self.nirs.as_mut() {
            Some(nirs) => nirs.clear(),
            _ => (),
        }
    }

    pub fn point_count(&self) -> usize {
        self.xyz.len() / 12
    }

    pub fn xyz(&self) -> &[u8] {
        &self.xyz
    }

    pub fn intensities(&self) -> &[u8] {
        &self.intensities
    }

    pub fn bit_attributes(&self) -> &[u8] {
        &self.bit_attributes
    }

    pub fn classifications(&self) -> &[u8] {
        &self.classifications
    }

    pub fn scan_angle_ranks(&self) -> &[u8] {
        &self.scan_angle_ranks
    }

    pub fn user_data(&self) -> &[u8] {
        &self.user_data
    }

    pub fn source_ids(&self) -> &[u8] {
        &self.source_ids
    }

    pub fn extra_bytes(&self) -> &[u8] {
        &self.extra_bytes
    }

    pub fn rgbs(&self) -> Option<&[u8]> {
        self.rgbs.as_ref().map(|rgb| &rgb[..])
    }

    pub fn gps_times(&self) -> Option<&[u8]> {
        self.gps_times.as_ref().map(|rgb| &rgb[..])
    }

    pub fn waveforms(&self) -> Option<&[u8]> {
        self.waveforms.as_ref().map(|rgb| &rgb[..])
    }

    pub fn nirs(&self) -> Option<&[u8]> {
        self.nirs.as_ref().map(|rgb| &rgb[..])
    }

    /**
     * Number of attributes that are defined in this LasPoints buffer
     */
    pub fn number_of_attributes(&self) -> u32 {
        let mut num_attributes = 8;
        if self.rgbs.is_some() {
            num_attributes += 1;
        }
        if self.gps_times.is_some() {
            num_attributes += 1;
        }
        if self.waveforms.is_some() {
            num_attributes += 1;
        }
        if self.nirs.is_some() {
            num_attributes += 1;
        }
        num_attributes
    }
}
