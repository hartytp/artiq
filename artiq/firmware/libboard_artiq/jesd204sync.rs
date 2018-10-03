use board_misoc::{csr, config, clock};

use hmc830_7043::delay_line;
use ad9154;

fn sysref_cal_dac(dacno: u8) -> Result<u16, &'static str> {

    print!("synchronising DAC {}...", dacno);
    let mut d = 255 as u16;
    let mut dmin = d;
    let mut dmax = d;

    dmin = d;
    dmax = d;
    delay_line::set_delay(0, d);
    delay_line::set_delay(1, d);
    clock::spin_us(10000);
    ad9154::dac_arm_sync(dacno);

    // measure negative margin
    loop {
        delay_line::set_delay(0, dmin);
        delay_line::set_delay(1, dmin);
        clock::spin_us(1000);

        let mut synchronised = true;
        for _ in 0..50 {
            if ad9154::check_dac_sync(dacno) != 0 {
                synchronised = false;
                break;
            }
        }
        if !synchronised { break; }
        dmin -= 1;
    }

    // measure positive margin
    loop {
        delay_line::set_delay(0, dmax);
        delay_line::set_delay(1, dmax);
        clock::spin_us(1000);

        let mut synchronised = true;
        for _ in 0..50 {
            if ad9154::check_dac_sync(dacno) != 0 {
                synchronised = false;
                break;
            }
        }
        if !synchronised { break; }
        dmax += 1;
    }

    if dmax - dmin > 10 {
        delay_line::set_delay(dacno, d);
        println!("success - dmin={}, dmax={}", dmin, dmax);

        info!("validation eye scan...");
        for delay in (dmin-10)..(dmax+10) {
            delay_line::set_delay(dacno, delay);
            print!("{}:", delay);
            for _ in 0..50 { print!(" {}", ad9154::check_dac_sync(dacno)); }
            print!("\n");
        }              

        return Ok(d);
    }
    println!("failed - dmin={}, dmax={}", dmin, dmax);
    loop { }
}

pub fn sysref_auto_dac_align() -> Result<(), &'static str> {
    // We assume that DAC SYSREF traces are length-matched so only one phase
    // value is needed, and we use DAC-0 as calibration reference.

    // let entry = config::read_str("sysref_phase_dac", |r| r.map(|s| s.parse()));
    // let phase = match entry {
    //   Ok(Ok(phase)) => phase,
    //    _ => {
    let phase = sysref_cal_dac(1)?;
    //      if let Err(e) = config::write_int("sysref_phase_dac", phase as u32) {
    //            error!("failed to update DAC SYSREF phase in config: {}", e);
    //        }
    //        phase
    //    }
    // };

    // for dacno in 0..csr::AD9154.len() {
    //sysref_dac_align(1 as u8, phase)?;
    // }
    Ok(())
}
