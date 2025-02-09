#include "esphome.h"
#include "esphome/components/sensor/sensor.h"
#include "OpenTherm.h"
#include "opentherm_switch.h"
#include "opentherm_climate.h"
#include "opentherm_binary.h"
#include "opentherm_output.h"

// Pins to OpenTherm Adapter
int inPin = D2; 
int outPin = D1;
OpenTherm ot(inPin, outPin, false);

IRAM_ATTR void handleInterrupt() {
  ot.handleInterrupt();
}

class OpenthermComponent: public PollingComponent {
private:
  const char *TAG = "opentherm_component";
  OpenthermFloatOutput *pid_output_;

  bool filtered_flame_state = false;
  int flame_off_counter = 0;
  const int flame_off_threshold = 2;

  // Helper function: sends a READ request for the given message ID
  inline float readOpenTherm(OpenThermMessageID message_id) {
    unsigned long request = ot.buildRequest(OpenThermRequestType::READ, message_id, 0);
    unsigned long response = ot.sendRequest(request);
    return ot.isValidResponse(response) ? ot.getFloat(response) : NAN;
  }

  // Helper function: sends a WRITE request for the given message ID
  inline bool writeOpenTherm(OpenThermMessageID message_id, float temperature) {
    unsigned int data = ot.temperatureToData(temperature);
    unsigned long request = ot.buildRequest(OpenThermRequestType::WRITE, message_id, data);
    unsigned long response = ot.sendRequest(request);
    return ot.isValidResponse(response);
  }

  // Helper function: publishes the sensor value if it is not NAN
  inline void publishIfValid(Sensor *sensor, float value) {
    if (!isnan(value)) {
      sensor->publish_state(value);
    }
  }

public:

  Switch *thermostatSwitch                  = new OpenthermSwitch();
  Sensor *external_temperature_sensor       = new Sensor();
  Sensor *return_temperature_sensor         = new Sensor();
  Sensor *boiler_temperature                = new Sensor();
  Sensor *pressure_sensor                   = new Sensor();
  Sensor *modulation_sensor                 = new Sensor();
  Sensor *heating_target_temperature_sensor = new Sensor();
  Sensor *dhw_flow_rate_sensor              = new Sensor();

  OpenthermClimate *hotWaterClimate     = new OpenthermClimate();
  OpenthermClimate *heatingWaterClimate = new OpenthermClimate();
  BinarySensor *flame                   = new OpenthermBinarySensor();

  // Set 3 sec. to give time to read all sensors (and not appear in HA as not available)
  OpenthermComponent(): PollingComponent(3000) { }

  void set_pid_output(OpenthermFloatOutput *pid_output) { pid_output_ = pid_output; }

  void setup() override {
    // This will be called once to set up the component
    // think of it as the setup() call in Arduino
    ESP_LOGD(TAG, "Setup");

    ot.begin(handleInterrupt);

    thermostatSwitch->add_on_state_callback([=](bool state) -> void {
      ESP_LOGD(TAG, "termostatSwitch_on_state_callback %d", state);
    });

    // Adjust HeatingWaterClimate depending on PID
    // heatingWaterClimate->set_supports_heat_cool_mode(this->pid_output_ != nullptr);
    heatingWaterClimate->set_supports_two_point_target_temperature(this->pid_output_ != nullptr);

    hotWaterClimate->set_temperature_settings(5, 6, 40);
    heatingWaterClimate->set_temperature_settings(35, 65, 0);
    hotWaterClimate->setup();
    heatingWaterClimate->setup();
  }

  void update() override {
    ESP_LOGD(TAG, "update heatingWaterClimate: %i", heatingWaterClimate->mode);
    ESP_LOGD(TAG, "update hotWaterClimate: %i", hotWaterClimate->mode);

    bool enableCentralHeating = (heatingWaterClimate->mode == ClimateMode::CLIMATE_MODE_HEAT);
    bool enableHotWater = (hotWaterClimate->mode == ClimateMode::CLIMATE_MODE_HEAT);
    bool enableCooling = false; // this boiler is for heating only

    // Set/Get Boiler Status
    auto response = ot.setBoilerStatus(enableCentralHeating, enableHotWater, enableCooling);

    if (!ot.isValidResponse(response)) {
      ESP_LOGW(TAG, "No communication with boiler detected!");
      return;  // exit update to avoid bootloop
    }

    // bool isFlameOn = ot.isFlameOn(response);
    bool current_flame_state = ot.isFlameOn(response);  

    // Filter the reading for the burner – ignore short fluctuations
    if (!current_flame_state) {
      flame_off_counter++;
    } else {
      flame_off_counter = 0;
    }
    // If the counter reaches the threshold, we consider the burner to have actually turned off; otherwise, we keep the previous "on" state
    bool new_filtered_state = (flame_off_counter >= flame_off_threshold) ? false : true;
    if (new_filtered_state != filtered_flame_state) {
      filtered_flame_state = new_filtered_state;
      flame->publish_state(filtered_flame_state);
    }

    bool isCentralHeatingActive = ot.isCentralHeatingActive(response);
    bool isHotWaterActive = ot.isHotWaterActive(response);

    float return_temperature    = readOpenTherm(OpenThermMessageID::Tret);
    float ext_temperature       = readOpenTherm(OpenThermMessageID::Toutside);
    float hotWater_temperature  = readOpenTherm(OpenThermMessageID::Tdhw);
    float pressure              = readOpenTherm(OpenThermMessageID::CHPressure);
    float modulation            = readOpenTherm(OpenThermMessageID::RelModLevel);
    float dhw_flow_rate         = readOpenTherm(OpenThermMessageID::DHWFlowRate);

    // Read boiler temperature – consider the reading valid if it is > 0
    float tempBoilerTemperature = ot.getBoilerTemperature();
    float boilerTemperature = (tempBoilerTemperature > 0) ? tempBoilerTemperature : NAN;

    // Set temperature depending on room thermostat
    float heating_target_temperature = NAN;
    if (this->pid_output_ != nullptr) {
      float pid_output = pid_output_->get_state();
      if (pid_output == 0.0f) {
        heating_target_temperature = 0.0f;
      } else {
        heating_target_temperature = pid_output * (heatingWaterClimate->target_temperature_high - heatingWaterClimate->target_temperature_low) 
                                      + heatingWaterClimate->target_temperature_low;
      }
      ESP_LOGD(TAG, "setBoilerTemperature at %f °C (from PID Output)", heating_target_temperature);
    } else if (thermostatSwitch->state) {
      heating_target_temperature = heatingWaterClimate->target_temperature;
      ESP_LOGD(TAG, "setBoilerTemperature at %f °C (from heating water climate)", heating_target_temperature);
    } else {
      heating_target_temperature = 0.0;
      ESP_LOGD(TAG, "setBoilerTemperature at %f °C (default low value)", heating_target_temperature);
    }
    ot.setBoilerTemperature(heating_target_temperature);

    // Set hot water temperature
    writeOpenTherm(OpenThermMessageID::TdhwSet, hotWaterClimate->target_temperature);

    // Publish readings – only send valid values
    // flame->publish_state(isFlameOn);
    publishIfValid(external_temperature_sensor, ext_temperature);
    publishIfValid(return_temperature_sensor, return_temperature);
    publishIfValid(boiler_temperature, boilerTemperature);
    publishIfValid(pressure_sensor, pressure);
    publishIfValid(modulation_sensor, modulation);
    publishIfValid(heating_target_temperature_sensor, heating_target_temperature);
    publishIfValid(dhw_flow_rate_sensor, dhw_flow_rate);

    // Publish status of thermostat that controls hot water
    if (!isnan(hotWater_temperature))
      hotWaterClimate->current_temperature = hotWater_temperature;
    hotWaterClimate->action = isHotWaterActive ? ClimateAction::CLIMATE_ACTION_HEATING : ClimateAction::CLIMATE_ACTION_OFF;
    hotWaterClimate->publish_state();

    // Publish status of thermostat that controls heating
    if (!isnan(boilerTemperature))
      heatingWaterClimate->current_temperature = boilerTemperature;
    heatingWaterClimate->action = (isCentralHeatingActive && filtered_flame_state) ? ClimateAction::CLIMATE_ACTION_HEATING : ClimateAction::CLIMATE_ACTION_OFF;
    heatingWaterClimate->publish_state();
  }

};