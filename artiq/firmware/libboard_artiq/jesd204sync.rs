use board_misoc::{csr, config};

use hmc830_7043::hmc7043;
use ad9154;

fn sysref_cal_dac(dacno: u8) -> Result<u16, &'static str> {
    info!("calibrating SYSREF phase at DAC-{}...", dacno);

    let mut d = 0;
    let dmin;
    let dmax;

    hmc7043::sysref_offset_dac(dacno, d);
    ad9154::dac_sync(dacno)?;

    loop {
        hmc7043::sysref_offset_dac(dacno, d);
        let realign_occured = ad9154::dac_sync(dacno)?;
        if realign_occured {
            dmin = d;
            break;
        }

        d += 1;
        if d > 128 {
            return Err("no sync errors found when scanning delay");
        }
    }

    d += 17;  // get away from jitter
    hmc7043::sysref_offset_dac(dacno, d);
    ad9154::dac_sync(dacno)?;

    loop {
        hmc7043::sysref_offset_dac(dacno, d);
        let realign_occured = ad9154::dac_sync(dacno)?;
        if realign_occured {
            dmax = d;
            break;
        }

        d += 1;
        if d > 128 {
            return Err("no sync errors found when scanning delay");
        }
    }

    let phase = (dmin+dmax)/2;
    info!("  ...done, min={}, max={}, result={}", dmin, dmax, phase);
    Ok(phase)
}

fn sysref_dac_align(dacno: u8, phase: u16) -> Result<(), &'static str> {
    let mut margin_minus = None;
    let mut margin_plus = None;

    info!("verifying SYSREF margins at DAC-{}...", dacno);

    hmc7043::sysref_offset_dac(dacno, phase);
    ad9154::dac_sync(dacno)?;
    for d in 0..128 {
        hmc7043::sysref_offset_dac(dacno, phase - d);
        let realign_occured = ad9154::dac_sync(dacno)?;
        if realign_occured {
            margin_minus = Some(d);
            break;
        }
    }

    hmc7043::sysref_offset_dac(dacno, phase);
    ad9154::dac_sync(dacno)?;
    for d in 0..128 {
        hmc7043::sysref_offset_dac(dacno, phase + d);
        let realign_occured = ad9154::dac_sync(dacno)?;
        if realign_occured {
            margin_plus = Some(d);
            break;
        }
    }

    if margin_minus.is_some() && margin_plus.is_some() {
        let margin_minus = margin_minus.unwrap();
        let margin_plus = margin_plus.unwrap();
        info!("  margins: -{} +{}", margin_minus, margin_plus);
        if margin_minus < 10 || margin_plus < 10 {
            return Err("SYSREF margins at DAC are too small, board needs recalibration");
        }
    } else {
        return Err("Unable to determine SYSREF margins at DAC");
    }

    // Put SYSREF at the correct phase and sync DAC
    hmc7043::sysref_offset_dac(dacno, phase);
    ad9154::dac_sync(dacno)?;

    Ok(())
}

pub fn sysref_auto_dac_align() -> Result<(), &'static str> {
    // We assume that DAC SYSREF traces are length-matched so only one phase
    // value is needed, and we use DAC-0 as calibration reference.

    let entry = config::read_str("sysref_phase_dac", |r| r.map(|s| s.parse()));
    let phase = match entry {
        Ok(Ok(phase)) => phase,
        _ => {
            let phase = sysref_cal_dac(0)?;
            if let Err(e) = config::write_int("sysref_phase_dac", phase as u32) {
                error!("failed to update DAC SYSREF phase in config: {}", e);
            }
            phase
        }
    };
    unsafe {
        csr::ad9154_crg::jref_en_write(1);
    }

    for dacno in 0..csr::AD9154.len() {
        sysref_dac_align(dacno as u8, phase)?;
    }
    Ok(())
}
