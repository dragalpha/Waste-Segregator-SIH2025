// mqtt.js — connect to MQTT broker, subscribe to telemetry topic and re-publish commands
const mqtt = require('mqtt');
const EventEmitter = require('events');

const config = require('./config/mqttConfig');

const emitter = new EventEmitter();
let client = null;

function start() {
  // Prevent multiple connections
  if (client && client.connected) {
    console.log('MQTT client already connected');
    return;
  }

  // Close existing client if it exists but not connected
  if (client) {
    client.end();
  }

  const url = config.url || 'mqtt://localhost:1883';
  const options = {
    ...config.options,
    reconnectPeriod: 1000, // Reconnect every 1 second
    connectTimeout: 30 * 1000 // 30 seconds timeout
  };
  client = mqtt.connect(url, options);

  client.on('connect', () => {
    console.log('MQTT client connected');
    console.log('Connected to MQTT broker:', url);
    // subscribe to telemetry topic
    const telemetryTopic = (config.topics && config.topics.telemetry) || 'pavitrax/boat1/telemetry';
    client.subscribe(telemetryTopic, { qos: 0 }, (err) => {
      if (!err) console.log('Subscribed to', telemetryTopic);
      else console.error('Subscribe error', err);
    });
  });

  client.on('reconnect', () => {
    console.log('Reconnecting to MQTT broker...');
  });

  client.on('close', () => {
    console.log('MQTT connection closed');
  });

  client.on('offline', () => {
    console.log('MQTT client went offline');
  });

  client.on('message', (topic, message) => {
    try {
      const payload = message.toString();
      // attempt parse JSON, fallback to raw
      let data;
      try { data = JSON.parse(payload); } catch(e) { data = { raw: payload }; }
      // emit an event so backend can handle/store/forward
      emitter.emit('telemetry', { topic, data, receivedAt: new Date().toISOString() });
    } catch (err) {
      console.error('MQTT message handling error', err);
    }
  });

  client.on('error', (err) => {
    console.error('MQTT error', err);
  });
}

function publish(topic, message) {
  if (!client) return console.error('MQTT client not connected');
  client.publish(topic, typeof message === 'string' ? message : JSON.stringify(message));
}

module.exports = {
  start,
  publish,
  on: (event, cb) => emitter.on(event, cb),
  get topics() {
    return config.topics || {
      telemetry: 'pavitrax/boat1/telemetry',
      commands: 'pavitrax/boat1/commands'
    };
  }
};
