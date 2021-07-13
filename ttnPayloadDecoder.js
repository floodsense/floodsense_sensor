// decoder variable contains Error Flags followed by Battery and sensor data.
function Decoder(b, port) {
  var decoded = {};

  // Error Flags
  /*
  Format:
    |--------------------------------------------- Error Flags  -----------------------------------------------------------------------------------------------------------|
    |     bit 7                                                    |     bit 6   |     bit 5    |     bit 4    |     bit 3    |     bit 2    |     bit 1    |     bit 0    |
    |     Used only for CFG update (all other bits are high)       |             |              |              |              |              |              | SD error flag|
  255 or FF is a CFG update
  */
  var errorFlag = b[0];
  if (errorFlag == 255) {
    // Payload is Sensor cfg update

    // Duty cycle
    var dutyCycle = (b[2] << 8) | b[1];
    decoded.dutyCycle = dutyCycle;

    // Sensor Mode
    var sensorMode = b[3];
    decoded.sensorMode = sensorMode;

    // Sensor Sampling Rate
    var sensorSamplingRate = (b[5] << 8) | b[4];
    decoded.sensorSamplingRate = sensorSamplingRate;

    // Sensor number of readings per measurement
    var sensorNumberOfReadings = b[6];
    decoded.sensorNumberOfReadings = sensorNumberOfReadings;

  } else {
    // Regular Payload
    var sdError, battery, distance;
    // Converting Error Flag bits
    sdError = errorFlag % 2;
    decoded.sdError = sdError;

    // battery
    battery = (b[2] << 8) | b[1]; // battery in centi Volts
    battery = battery / 1000; // Convert to Volts
    decoded.battery = battery;

    // distance
    distance = (b[4] << 8) | b[3];
    decoded.distance = distance;

    if (b[5] !== null) {
      // temperature
      var temperature = (b[6] & 0x80 ? 0xFFFF << 16 : 0) | b[6] << 8 | b[5];
      temperature = temperature / 100;
      decoded.temperature = temperature;

      // pressure
      var pressure = (((b[10] << 24) | (b[9] << 16)) | (b[8] << 8)) | b[7];
      pressure = pressure / 100;
      decoded.pressure = pressure;

      // altitude
      var altitude = (b[12] << 8) | b[11];
      altitude = altitude / 100;
      decoded.altitude = altitude;

      // humidity
      var humidity = (b[14] << 8) | b[13];
      humidity = humidity / 100;
      decoded.humidity = humidity;
    }
  }

  return decoded;
}
