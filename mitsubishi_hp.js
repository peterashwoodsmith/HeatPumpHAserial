//
// mitsubishi_hp.js - Zigbee2MQTT External Converter for Mitsubishi Heat Pump
//
// AUTHORS:
//   Arduino/ESP32 sketch: Peter Ashwood-Smith (c) 2026
//   Z2M Converter:        Peter Ashwood-Smith with Claude AI (c) 2026
//       AI promps:        Martin P Dee (to create and test this file)
//
// REPOSITORY:
//   https://github.com/peterashwoodsmith/HeatPumpHA
//
// INSTALLATION:
//   Place this file in your Zigbee2MQTT data directory under a folder
//   called 'external_converters', e.g:
//
//     Linux/Docker : <your_z2m_data_dir>/external_converters/mitsubishi_hp.js
//     Windows      : C:\Users\<user>\zigbee2mqtt\data\external_converters\mitsubishi_hp.js
//     HA Add-on    : /config/zigbee2mqtt/external_converters/mitsubishi_hp.js
//
//   No changes to configuration.yaml are needed for Z2M v2.0 and later.
//   For Z2M v1.x add to configuration.yaml:
//     external_converters:
//       - mitsubishi_hp.js
//
// REQUIREMENTS:
//   Zigbee2MQTT v2.0 or later
//   ESP32-C6 running the HeatPumpHA Arduino sketch
//
// TESTED WITH:
//   Zigbee2MQTT 2.4.0
//   Domoticz with built-in Z2M MQTT autodiscovery
//
// Zigbee Endpoint Map:
//   EP 10 - Binary Output - Power ON/OFF
//   EP 11 - Analog Output - Mode: 0=Auto,1=Cool,2=Dry,3=Heat,4=Fan Only
//   EP 12 - Analog Output - Temperature Setpoint (16-31°C)
//   EP 13 - Analog Output - Fan Speed (0=quiet,1-4=speed,5=auto)
//   EP 14 - Analog Output - Vane Position (0=auto,1-5=position,6=swing)
//   EP 17 - Analog Input  - Room Temperature (sensor)
//
// Domoticz Selector Switch Mapping:
//
//   AC Mode selector:
//     Level 0  = (off/null)
//     Level 10 = Auto        -> EP11 value 0 (not acted upon by HP)
//     Level 20 = Cool        -> EP11 value 1
//     Level 30 = Dehumidify  -> EP11 value 2 (not acted upon by HP)
//     Level 40 = Heat        -> EP11 value 3
//     Level 50 = Fan Only    -> EP11 value 4 (not acted upon by HP)
//
//   Fan Speed selector:
//     Level 0  = (off/null)
//     Level 10 = Auto        -> EP13 value 5
//     Level 20 = Silence     -> EP13 value 0
//     Level 30 = 1           -> EP13 value 1
//     Level 40 = 2           -> EP13 value 2
//     Level 50 = 3           -> EP13 value 3
//     Level 60 = 4           -> EP13 value 4
//
//   Vane selector:
//     Level 0  = (off/null)
//     Level 10 = Auto        -> EP14 value 0
//     Level 20 = 1           -> EP14 value 1
//     Level 30 = 2           -> EP14 value 2
//     Level 40 = 3           -> EP14 value 3
//     Level 50 = 4           -> EP14 value 4
//     Level 60 = 5           -> EP14 value 5
//     Level 70 = Oscillant   -> EP14 value 6
//

const e = require('zigbee-herdsman-converters/lib/exposes');
const ea = e.access;
const zhc = require('zigbee-herdsman-converters');

// Fan: map between EP13 raw value and selector text name
const fanValueToName = { 5: 'Auto', 0: 'Silence', 1: '1', 2: '2', 3: '3', 4: '4' };
const fanNameToValue = { 'Auto': 5, 'Silence': 0, '1': 1, '2': 2, '3': 3, '4': 4 };

// Vane: map between EP14 raw value and selector text name
const vaneValueToName = { 0: 'Auto', 1: '1', 2: '2', 3: '3', 4: '4', 5: '5', 6: 'Oscillant' };
const vaneNameToValue = { 'Auto': 0, '1': 1, '2': 2, '3': 3, '4': 4, '5': 5, 'Oscillant': 6 };

// Mode: map between EP11 numeric value and selector text name
const modeValueToName = { 0: 'Auto', 1: 'Cool', 2: 'Dehumidify', 3: 'Heat', 4: 'Fan Only' };
const modeNameToValue = { 'Auto': 0, 'Cool': 1, 'Dehumidify': 2, 'Heat': 3, 'Fan Only': 4 };

const definition = {
    zigbeeModel: ['Z2MS004'],
    model: 'Z2MS004',
    vendor: 'RiverView',
    description: 'Mitsubishi Heat Pump Zigbee Bridge',

    fromZigbee: [
        {
            // EP10 = Power, EP11 = Mode
            cluster: 'genBinaryOutput',
            type: ['attributeReport', 'readResponse'],
            convert: (model, msg, publish, options, meta) => {
                const ep = msg.endpoint.ID;
                if (ep === 10) {
                    return { power: msg.data.presentValue ? 'ON' : 'OFF' };
                }
            },
        },
        {
            // EP11 = Mode (now analog), EP12 = Setpoint, EP13 = Fan, EP14 = Vane
            cluster: 'genAnalogOutput',
            type: ['attributeReport', 'readResponse'],
            convert: (model, msg, publish, options, meta) => {
                const ep = msg.endpoint.ID;
                if (ep === 11) {
                    return { ac_mode: modeValueToName[msg.data.presentValue] || 'Heat' };
                }
                if (ep === 12) {
                    return { occupied_heating_setpoint: msg.data.presentValue };
                }
                if (ep === 13) {
                    return { fan_mode: fanValueToName[msg.data.presentValue] || 'Auto' };
                }
                if (ep === 14) {
                    return { vane_mode: vaneValueToName[msg.data.presentValue] || 'Auto' };
                }
            },
        },
        {
            // EP17 = Room Temperature
            cluster: 'genAnalogInput',
            type: ['attributeReport', 'readResponse'],
            convert: (model, msg, publish, options, meta) => {
                const ep = msg.endpoint.ID;
                if (ep === 17) {
                    return { local_temperature: msg.data.presentValue };
                }
            },
        },
    ],

    toZigbee: [
        {
            // Power ON/OFF -> EP10
            key: ['power'],
            convertSet: async (entity, key, value, meta) => {
                const ep = meta.device.getEndpoint(10);
                await ep.write('genBinaryOutput', { presentValue: value === 'ON' ? 1 : 0 });
                return { state: { power: value } };
            },
        },
        {
            // AC Mode selector name -> EP11 (analog 0-4)
            key: ['ac_mode'],
            convertSet: async (entity, key, value, meta) => {
                const ep = meta.device.getEndpoint(11);
                const modeValue = modeNameToValue[value] !== undefined ? modeNameToValue[value] : 3;
                await ep.write('genAnalogOutput', { presentValue: modeValue });
                return { state: { ac_mode: value } };
            },
        },
        {
            // Temperature setpoint -> EP12
            key: ['occupied_heating_setpoint'],
            convertSet: async (entity, key, value, meta) => {
                const ep = meta.device.getEndpoint(12);
                await ep.write('genAnalogOutput', { presentValue: value });
                return { state: { occupied_heating_setpoint: value } };
            },
        },
        {
            // Fan selector name -> EP13
            key: ['fan_mode'],
            convertSet: async (entity, key, value, meta) => {
                const ep = meta.device.getEndpoint(13);
                const fanValue = fanNameToValue[value] !== undefined ? fanNameToValue[value] : 5;
                await ep.write('genAnalogOutput', { presentValue: fanValue });
                return { state: { fan_mode: value } };
            },
        },
        {
            // Vane selector name -> EP14
            key: ['vane_mode'],
            convertSet: async (entity, key, value, meta) => {
                const ep = meta.device.getEndpoint(14);
                const vaneValue = vaneNameToValue[value] !== undefined ? vaneNameToValue[value] : 0;
                await ep.write('genAnalogOutput', { presentValue: vaneValue });
                return { state: { vane_mode: value } };
            },
        },
    ],

    exposes: [
        // Power on/off switch
        e.binary('power', ea.STATE_SET, 'ON', 'OFF')
            .withDescription('Power on/off'),

        // AC Mode selector
        e.enum('ac_mode', ea.STATE_SET, ['Auto', 'Cool', 'Dehumidify', 'Heat', 'Fan Only'])
            .withDescription('AC Mode'),

        // Temperature setpoint via climate entity so Domoticz creates a setpoint device
        e.climate()
            .withSetpoint('occupied_heating_setpoint', 16, 31, 1)
            .withLocalTemperature(),

        // Fan speed selector
        e.enum('fan_mode', ea.STATE_SET, ['Auto', 'Silence', '1', '2', '3', '4'])
            .withDescription('Fan speed'),

        // Vane position selector
        e.enum('vane_mode', ea.STATE_SET, ['Auto', '1', '2', '3', '4', '5', 'Oscillant'])
            .withDescription('Vane position'),
    ],

    configure: async (device, coordinatorEndpoint, logger) => {
        // Power - EP10 binary output
        const ep10 = device.getEndpoint(10);
        await ep10.bind('genBinaryOutput', coordinatorEndpoint);
        await ep10.configureReporting('genBinaryOutput', [{
            attribute: 'presentValue',
            minimumReportInterval: 0,
            maximumReportInterval: 300,
            reportableChange: 1,
        }]);
        // Mode, Setpoint, Fan, Vane - EP11-14 analog outputs
        for (const epId of [11, 12, 13, 14]) {
            const ep = device.getEndpoint(epId);
            await ep.bind('genAnalogOutput', coordinatorEndpoint);
            await ep.configureReporting('genAnalogOutput', [{
                attribute: 'presentValue',
                minimumReportInterval: 0,
                maximumReportInterval: 300,
                reportableChange: 1,
            }]);
        }
        // Room temperature - EP17 analog input
        const ep17 = device.getEndpoint(17);
        await ep17.bind('genAnalogInput', coordinatorEndpoint);
        await ep17.configureReporting('genAnalogInput', [{
            attribute: 'presentValue',
            minimumReportInterval: 30,
            maximumReportInterval: 300,
            reportableChange: 0.1,
        }]);
    },
};

zhc.addExternalDefinition(definition);
module.exports = definition;