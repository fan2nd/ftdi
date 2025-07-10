use crate::ftdaye::FtdiError;
use crate::mpsse_cmd::MpsseCmdBuilder;
use crate::{FtdiMpsse, Pin, PinUse};
use std::sync::{Arc, Mutex};

/// FTDI output pin.
pub struct OutputPin {
    /// Parent FTDI device.
    mtx: Arc<Mutex<FtdiMpsse>>,
    /// GPIO pin index.
    pin: Pin,
}

impl Drop for OutputPin {
    fn drop(&mut self) {
        let mut lock = self.mtx.lock().unwrap();
        lock.free_pin(self.pin);
    }
}

impl OutputPin {
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>, pin: Pin) -> Result<OutputPin, FtdiError> {
        if pin == Pin::Lower(2) {
            return Err(FtdiError::Other(format!(
                "In mpsse mode {pin:?} can not be use as input according to AN108-2.1."
            )));
        }
        {
            let mut lock = mtx.lock().unwrap();

            lock.alloc_pin(pin, PinUse::Output);

            let (byte, idx) = match pin {
                Pin::Lower(idx) => (&mut lock.lower, idx),
                Pin::Upper(idx) => (&mut lock.upper, idx),
            };
            byte.direction |= 1 << idx;
            let mut cmd = MpsseCmdBuilder::new();
            match pin {
                Pin::Lower(_) => cmd.set_gpio_lower(byte.value, byte.direction),
                Pin::Upper(_) => cmd.set_gpio_upper(byte.value, byte.direction),
            };
            lock.write_read(cmd.as_slice(), &mut [])?;
        }
        Ok(OutputPin { mtx, pin })
    }

    pub(crate) fn set(&self, state: bool) -> Result<(), FtdiError> {
        let mut lock = self.mtx.lock().unwrap();

        let byte = match self.pin {
            Pin::Lower(_) => &mut lock.lower,
            Pin::Upper(_) => &mut lock.upper,
        };

        if state {
            byte.value |= self.mask();
        } else {
            byte.value &= !self.mask();
        };

        let mut cmd = MpsseCmdBuilder::new();
        match self.pin {
            Pin::Lower(_) => cmd.set_gpio_lower(byte.value, byte.direction),
            Pin::Upper(_) => cmd.set_gpio_upper(byte.value, byte.direction),
        };
        lock.write_read(cmd.as_slice(), &mut [])?;

        Ok(())
    }
    /// Convert the GPIO pin index to a pin mask
    pub(crate) fn mask(&self) -> u8 {
        let idx = match self.pin {
            Pin::Lower(idx) => idx,
            Pin::Upper(idx) => idx,
        };
        1 << idx
    }
}

impl eh1::digital::Error for FtdiError {
    fn kind(&self) -> eh1::digital::ErrorKind {
        eh1::digital::ErrorKind::Other
    }
}

impl eh1::digital::ErrorType for OutputPin {
    type Error = FtdiError;
}

impl eh1::digital::OutputPin for OutputPin {
    fn set_low(&mut self) -> Result<(), FtdiError> {
        self.set(false)
    }

    fn set_high(&mut self) -> Result<(), FtdiError> {
        self.set(true)
    }
}

/// FTDI input pin.
pub struct InputPin {
    /// Parent FTDI device.
    mtx: Arc<Mutex<FtdiMpsse>>,
    /// GPIO pin index.
    pin: Pin,
}

impl Drop for InputPin {
    fn drop(&mut self) {
        let mut lock = self.mtx.lock().unwrap();
        lock.free_pin(self.pin);
    }
}

impl InputPin {
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>, pin: Pin) -> Result<InputPin, FtdiError> {
        if pin == Pin::Lower(0) || pin == Pin::Lower(1) || pin == Pin::Lower(3) || {
            return Err(FtdiError::Other(format!(
                "In mpsse mode {pin:?} can not be use as input according to AN108-2.1."
            )));
        } {
            let mut lock = mtx.lock().unwrap();

            lock.alloc_pin(pin, PinUse::Input);

            let (byte, idx) = match pin {
                Pin::Lower(idx) => (&mut lock.lower, idx),
                Pin::Upper(idx) => (&mut lock.upper, idx),
            };
            byte.direction &= !(1 << idx);
            let mut cmd = MpsseCmdBuilder::new();
            match pin {
                Pin::Lower(_) => cmd.set_gpio_lower(byte.value, byte.direction),
                Pin::Upper(_) => cmd.set_gpio_upper(byte.value, byte.direction),
            };
            lock.write_read(cmd.as_slice(), &mut [])?;
        }
        Ok(InputPin { mtx, pin })
    }

    pub(crate) fn get(&self) -> Result<bool, FtdiError> {
        let lock = self.mtx.lock().unwrap();

        let mut buffer = [0u8; 1];
        let mut cmd = MpsseCmdBuilder::new();
        match self.pin {
            Pin::Lower(_) => cmd.gpio_lower(),
            Pin::Upper(_) => cmd.gpio_upper(),
        };
        lock.write_read(cmd.as_slice(), &mut buffer)?;

        Ok((buffer[0] & self.mask()) != 0)
    }

    /// Convert the GPIO pin index to a pin mask
    pub(crate) fn mask(&self) -> u8 {
        let idx = match self.pin {
            Pin::Lower(idx) => idx,
            Pin::Upper(idx) => idx,
        };
        1 << idx
    }
}

impl eh1::digital::ErrorType for InputPin {
    type Error = FtdiError;
}

impl eh1::digital::InputPin for InputPin {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        self.get()
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        self.get().map(|res| !res)
    }
}
