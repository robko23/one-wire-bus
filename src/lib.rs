#![no_std]

use embassy_time::Timer;
use embedded_hal::digital::{InputPin, OutputPin};

mod address;
pub mod commands;
pub mod crc;
mod error;

pub use address::Address;
pub use error::{OneWireError, OneWireResult};

pub const READ_SLOT_DURATION_MICROS: u16 = 70;

/// Implementation of the 1-Wire protocol.
/// https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/126.html
#[derive(Debug)]
pub struct SearchState {
    // The address of the last found device
    address: u64,

    // bitflags of discrepancies found
    discrepancies: u64,

    // index of the last (leftmost / closest to MSB) discrepancy bit. This can be calculated from the
    // discrepancy bitflags, but it's cheaper to just save it. Index is an offset from the LSB
    last_discrepancy_index: u8,
}

pub struct OneWire<T> {
    pin: T,
}

impl<T, E> OneWire<T>
    where
        T: InputPin<Error=E>,
        T: OutputPin<Error=E>,
{
    pub fn new(pin: T) -> OneWireResult<OneWire<T>, E> {
        let mut one_wire = OneWire { pin };
        // Pin should be high during idle.
        one_wire.release_bus()?;
        Ok(one_wire)
    }

    pub fn into_inner(self) -> T {
        self.pin
    }

    /// Disconnects the bus, letting another device (or the pull-up resistor) set the bus value
    pub fn release_bus(&mut self) -> OneWireResult<(), E> {
        self.pin
            .set_high()
            .map_err(|err| OneWireError::PinError(err))
    }

    /// Drives the bus low
    pub fn set_bus_low(&mut self) -> OneWireResult<(), E> {
        self.pin
            .set_low()
            .map_err(|err| OneWireError::PinError(err))
    }

    pub fn is_bus_high(&self) -> OneWireResult<bool, E> {
        self.pin
            .is_high()
            .map_err(|err| OneWireError::PinError(err))
    }

    pub fn is_bus_low(&self) -> OneWireResult<bool, E> {
        self.pin.is_low().map_err(|err| OneWireError::PinError(err))
    }

    async fn wait_for_high(&self) -> OneWireResult<(), E> {
        // wait up to 250 µs for the bus to become high (from the pull-up resistor)
        for _ in 0..125 {
            if self.is_bus_high()? {
                return Ok(());
            }
            Timer::after_micros(2).await;
        }
        Err(OneWireError::BusNotHigh)
    }

    /// Sends a reset pulse, then returns true if a device is present
    pub async fn reset(&mut self) -> OneWireResult<bool, E> {
        self.wait_for_high().await?;

        self.set_bus_low()?;
        Timer::after_micros(480).await; // Maxim recommended wait time

        self.release_bus()?;
        Timer::after_micros(70).await; // Maxim recommended wait time

        let device_present = self.is_bus_low()?;

        Timer::after_micros(410).await; // Maxim recommended wait time
        Ok(device_present)
    }

    pub async fn read_bit(&mut self) -> OneWireResult<bool, E> {
        self.set_bus_low()?;
        Timer::after_micros(6).await; // Maxim recommended wait time

        self.release_bus()?;
        // Maxim recommended wait time
        Timer::after_micros(9).await;

        let bit_value = self.is_bus_high()?;
        // Maxim recommended wait time
        Timer::after_micros(55).await;
        Ok(bit_value)
    }

    pub async fn read_byte(&mut self) -> OneWireResult<u8, E> {
        let mut output: u8 = 0;
        for _ in 0..8 {
            output >>= 1;
            if self.read_bit().await? {
                output |= 0x80;
            }
        }
        Ok(output)
    }
    pub async fn read_bytes(
        &mut self,
        output: &mut [u8],
    ) -> OneWireResult<(), E> {
        for i in 0..output.len() {
            output[i] = self.read_byte().await?;
        }
        Ok(())
    }

    pub async fn write_1_bit(&mut self) -> OneWireResult<(), E> {
        self.set_bus_low()?;
        // Maxim recommended wait time
        Timer::after_micros(6).await;

        self.release_bus()?;
        // Maxim recommended wait time
        Timer::after_micros(64).await;
        Ok(())
    }

    pub async fn write_0_bit(&mut self) -> OneWireResult<(), E> {
        self.set_bus_low()?;
        // Maxim recommended wait time
        Timer::after_micros(60).await;

        self.release_bus()?;
        // Maxim recommended wait time
        Timer::after_micros(10).await;
        Ok(())
    }

    pub async fn write_bit(
        &mut self,
        value: bool,
    ) -> OneWireResult<(), E> {
        if value {
            self.write_1_bit().await
        } else {
            self.write_0_bit().await
        }
    }

    pub async fn write_byte(
        &mut self,
        mut value: u8,
    ) -> OneWireResult<(), E> {
        for _ in 0..8 {
            self.write_bit(value & 0x01 == 0x01).await?;
            value >>= 1;
        }
        Ok(())
    }

    pub async fn write_bytes(
        &mut self,
        bytes: &[u8],
    ) -> OneWireResult<(), E> {
        for i in 0..bytes.len() {
            self.write_byte(bytes[i]).await?;
        }
        Ok(())
    }

    /// Address a specific device. All others will wait for a reset pulse.
    /// This should only be called after a reset, and should be immediately followed by another command
    pub async fn match_address(
        &mut self,
        address: &Address,
    ) -> OneWireResult<(), E> {
        self.write_byte(commands::MATCH_ROM).await?;
        self.write_bytes(&address.0.to_le_bytes()).await?;
        Ok(())
    }

    /// Address all devices on the bus simultaneously.
    /// This should only be called after a reset, and should be immediately followed by another command
    pub async fn skip_address(&mut self) -> OneWireResult<(), E> {
        self.write_byte(commands::SKIP_ROM).await?;
        Ok(())
    }

    /// Sends a reset, followed with either a SKIP_ROM or MATCH_ROM (with an address), and then the supplied command
    /// This should be followed by any reading/writing, if needed by the command used
    pub async fn send_command(
        &mut self,
        command: u8,
        address: Option<&Address>,
    ) -> OneWireResult<(), E> {
        self.reset().await?;
        if let Some(address) = address {
            self.match_address(address).await?;
        } else {
            self.skip_address().await?;
        }
        self.write_byte(command).await?;
        Ok(())
    }

    /// Returns an iterator that iterates over all device addresses on the bus
    /// They can be filtered to only alarming devices if needed
    /// There is no requirement to immediately finish iterating all devices, but if devices are
    /// added / removed / change alarm state, the search may return an error or fail to find a device
    /// Device addresses will always be returned in the same order (lowest to highest, Little Endian)
    pub fn devices<D>(
        &mut self,
        only_alarming: bool,
    ) -> DeviceSearch<T>
    {
        DeviceSearch {
            onewire: self,
            state: None,
            finished: false,
            only_alarming,
        }
    }

    /// Search for device addresses on the bus
    /// They can be filtered to only alarming devices if needed
    /// Start the first search with a search_state of `None`, then use the returned state for subsequent searches
    /// There is no time limit for continuing a search, but if devices are
    /// added / removed / change alarm state, the search may return an error or fail to find a device
    /// Device addresses will always be returned in the same order (lowest to highest, Little Endian)
    pub async fn device_search(
        &mut self,
        search_state: Option<&SearchState>,
        only_alarming: bool,
    ) -> OneWireResult<Option<(Address, SearchState)>, E> {
        if let Some(search_state) = search_state {
            if search_state.discrepancies == 0 {
                return Ok(None);
            }
        }

        if !self.reset().await? {
            return Ok(None);
        }
        if only_alarming {
            self.write_byte(commands::SEARCH_ALARM).await?;
        } else {
            self.write_byte(commands::SEARCH_NORMAL).await?;
        }

        let mut last_discrepancy_index: u8 = 0;
        let mut address;
        let mut discrepancies;
        let continue_start_bit;

        if let Some(search_state) = search_state {
            // follow up to the last discrepancy
            for bit_index in 0..search_state.last_discrepancy_index {
                let _false_bit = !self.read_bit().await?;
                let _true_bit = !self.read_bit().await?;
                let was_discrepancy_bit =
                    (search_state.discrepancies & (1_u64 << (bit_index as u64))) != 0;
                if was_discrepancy_bit {
                    last_discrepancy_index = bit_index;
                }
                let previous_chosen_bit =
                    (search_state.address & (1_u64 << (bit_index as u64))) != 0;

                // choose the same as last time
                self.write_bit(previous_chosen_bit).await?;
            }
            address = search_state.address;
            // This is the discrepancy bit. False is always chosen to start, so choose true this time
            {
                let false_bit = !self.read_bit().await?;
                let true_bit = !self.read_bit().await?;
                if !(false_bit && true_bit) {
                    // A different response was received than last search
                    return Err(OneWireError::UnexpectedResponse);
                }
                let address_mask = 1_u64 << (search_state.last_discrepancy_index as u64);
                address |= address_mask;
                self.write_bit(true).await?;
            }

            //keep all discrepancies except the last one
            discrepancies = search_state.discrepancies
                & !(1_u64 << (search_state.last_discrepancy_index as u64));
            continue_start_bit = search_state.last_discrepancy_index + 1;
        } else {
            address = 0;
            discrepancies = 0;
            continue_start_bit = 0;
        }
        for bit_index in continue_start_bit..64 {
            let false_bit = !self.read_bit().await?;
            let true_bit = !self.read_bit().await?;
            let chosen_bit = match (false_bit, true_bit) {
                (false, false) => {
                    // No devices responded to the search request
                    return Err(OneWireError::UnexpectedResponse);
                }
                (false, true) => {
                    // All remaining devices have the true bit set
                    true
                }
                (true, false) => {
                    // All remaining devices have the false bit set
                    false
                }
                (true, true) => {
                    // Discrepancy, multiple values reported
                    // choosing the lower value here
                    discrepancies |= 1_u64 << (bit_index as u64);
                    last_discrepancy_index = bit_index;
                    false
                }
            };
            let address_mask = 1_u64 << (bit_index as u64);
            if chosen_bit {
                address |= address_mask;
            } else {
                address &= !address_mask;
            }
            self.write_bit(chosen_bit).await?;
        }
        crc::check_crc8(&address.to_le_bytes())?;
        Ok(Some((
            Address(address),
            SearchState {
                address,
                discrepancies,
                last_discrepancy_index,
            },
        )))
    }
}

pub struct DeviceSearch<'a, T> {
    onewire: &'a mut OneWire<T>,
    state: Option<SearchState>,
    finished: bool,
    only_alarming: bool,
}

impl<'a, T, E> DeviceSearch<'a, T>
    where
        T: InputPin<Error=E>,
        T: OutputPin<Error=E>
{
    pub async fn next(&mut self) -> Option<OneWireResult<Address, E>> {
        if self.finished {
            return None;
        }
        let result =
            self.onewire
                .device_search(self.state.as_ref(), self.only_alarming).await;
        match result {
            Ok(Some((address, search_state))) => {
                self.state = Some(search_state);
                Some(Ok(address))
            }
            Ok(None) => {
                self.state = None;
                self.finished = true;
                None
            }
            Err(err) => {
                self.state = None;
                self.finished = true;
                Some(Err(err))
            }
        }
    }
}