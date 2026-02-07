# Specialised Guide for the SFM3003 Anemometer Logger
_See also the SFM3003 Sensor Guide._

Settings in the "Main" section controls the sensor-specific firmware to acquire data according to three different patterns:

1. Single Shot - one reading is taken per wake-up. The temperature is taken immediately and the flow after a warm-up time, as recommended in the datasheet.
2. Burst - the sensor is put into measurement mode and a stream of readings logged. The sensor remains "on" throughout and self-warming is evident. The LED flash is still very brief and occurs after the burst completes.
3. Averaged - a low power co-processor takes single readings, several of which are used to compute and record a mean at each main wake-up. This pattern has additional settings as described in the SFM3003 Sensor Guide.

Settings meaning/use:

- SFM\_BURST\_LEN = the number of readings in each burst, or 0 for Single Shot.
- SFM\_BPERIOD\_S = the interval between each reading within a burst. Take care that SFM\_BURST\_LEN x SFM\_BPERIOD\_S should be less than APP\_LOOP\_S.
- SFM\_USE\_LP\_CORE = whether to use the low power co-processor, ie to use Averaged pattern. If "y", this takes priority over the burst settings.

Note that in burst mode, the logger is busy for SFM\_BURST\_LEN x SFM\_BPERIOD\_S seconds; during this time the push button will not be active so you may have to wait before being able to start the WiFi access point.
