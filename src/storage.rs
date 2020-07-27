////! This file contains code used to [de]serialize calibration
////! points (etc) into byte arrays, for use with storing on
////! flash chips etc.
//
//use crate::{CalPt, OrpCalPt, EcCalPt};
//
//impl CalPt {
//    pub fn to_bytes(&self) -> [u8; 37] {
//        let mut r = [0; 37];
//
//        let mut addr = 0;
//
//        // todo: DRY
//        for byte in &float_to_bytes(self.ph_cal.0.V) {
//            r[addr] = *byte;
//            addr += 1;
//        }
//        for byte in &float_to_bytes(self.ph_cal.0.pH) {
//            r[addr] = *byte;
//            addr += 1;
//        }
//        for byte in &float_to_bytes(self.ph_cal.0.T) {
//            r[addr] = *byte;
//            addr += 1;
//        }
//
//        for byte in &float_to_bytes(self.ph_cal.1.V) {
//            r[addr] = *byte;
//            addr += 1;
//        }
//        for byte in &float_to_bytes(self.ph_cal.1.pH) {
//            r[addr] = *byte;
//            addr += 1;
//        }
//        for byte in &float_to_bytes(self.ph_cal.1.T) {
//            r[addr] = *byte;
//            addr += 1;
//        }
//
//        match self.ph_cal.2 {
//            Some(c) => {
//                r[addr] = 1;
//                addr += 1;
//
//                for byte in &float_to_bytes(c.V) {
//                    r[addr] = *byte;
//                    addr += 1;
//                }
//                for byte in &float_to_bytes(c.pH) {
//                    r[addr] = *byte;
//                    addr += 1;
//                }
//                for byte in &float_to_bytes(c.T) {
//                    r[addr] = *byte;
//                    addr += 1;
//                }
//            }
//            None => {
//                r[addr] = 0;
//                addr += 13;
//            }
//        }
//
//        r
//    }
//
//}
