// PAVITRAX Frontend JavaScript
const API_BASE = location.origin.includes('3000')
  ? location.origin.replace('3000', '4000')
  : location.origin;

let map, locationMap, gpsMap, zonesMap, pollutionMap;
let marker, locationMarker, gpsMarker;
let heatLayer;
let socket;
let charts = {};
let cameraStream;
let cameraDevices = [];
let cameraDeviceId = null;
const YOLO_STREAM_URL = 'http://127.0.0.1:8090/stream';

// Navigation
function initNavigation() {
  document.getElementById('nav').addEventListener('click', (e) => {
    if (e.target.tagName === 'BUTTON' || e.target.closest('button')) {
      const btn = e.target.closest('button') || e.target;
      const target = btn.dataset.target;
      if (target) {
        setActiveSection(target);
      }
    }
  });
}

function setActiveSection(target) {
  document.querySelectorAll('nav button').forEach(btn => {
    btn.classList.toggle('active', btn.dataset.target === target);
  });
  document.querySelectorAll('main section').forEach(sec => {
    sec.classList.toggle('active', sec.id === target);
  });
  
  // Initialize section-specific features
  if (target === 'dashboard') {
    initDashboardMaps();
    initYoloStream();
    startCamera(); // keep getUserMedia available if needed
  } else if (target === 'live') {
    initLiveMaps();
    startCamera(); // ensure camera is running when live control is active
  } else if (target === 'heatmap') {
    initHeatmap();
  } else if (target === 'predictions') {
    initPredictions();
  }
}

// Camera handling
function updateCameraStatus(isActive, message) {
  const indicators = document.querySelectorAll('.camera-status-text, .streaming-indicator');
  indicators.forEach((el) => {
    if (el.classList.contains('streaming-indicator')) {
      el.style.color = isActive ? '#ef4444' : '#6b7280';
      el.querySelector('.red-dot')?.classList.toggle('paused', !isActive);
      el.querySelector('.status-label')?.classList.toggle('muted', !isActive);
      if (message) el.setAttribute('data-tooltip', message);
    } else {
      el.textContent = isActive ? 'Camera On' : 'Camera Off';
      if (message) el.title = message;
    }
  });
}

function showCameraError(message) {
  const placeholders = document.querySelectorAll('.camera-placeholder, .live-camera-placeholder');
  placeholders.forEach((el) => {
    el.textContent = message;
    el.classList.add('error');
  });
  updateCameraStatus(false, message);
}

async function listCameraDevices(requestAccess = false) {
  if (!navigator.mediaDevices?.enumerateDevices) return [];
  try {
    if (requestAccess && navigator.mediaDevices.getUserMedia) {
      // Probe once to surface labels in some browsers
      const probe = await navigator.mediaDevices.getUserMedia({ video: true });
      probe.getTracks().forEach((t) => t.stop());
    }
    const devices = await navigator.mediaDevices.enumerateDevices();
    cameraDevices = devices.filter((d) => d.kind === 'videoinput');
    populateCameraSelect();
    return cameraDevices;
  } catch (err) {
    console.error('enumerateDevices failed:', err);
    return [];
  }
}

function populateCameraSelect() {
  const select = document.getElementById('camera-select');
  if (!select) return;
  select.innerHTML = '';
  cameraDevices.forEach((dev, idx) => {
    const opt = document.createElement('option');
    opt.value = dev.deviceId;
    opt.textContent = dev.label || `Camera ${idx + 1}`;
    opt.selected = cameraDeviceId ? dev.deviceId === cameraDeviceId : idx === 0;
    select.appendChild(opt);
  });
  if (!cameraDevices.length) {
    const opt = document.createElement('option');
    opt.textContent = 'No cameras found';
    opt.disabled = true;
    opt.selected = true;
    select.appendChild(opt);
  }
}

async function startCamera(deviceId = null) {
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    showCameraError('Camera not supported in this browser.');
    return null;
  }

  if (cameraStream && deviceId === cameraDeviceId) {
    updateCameraStatus(true);
    return cameraStream;
  }

  if (cameraStream) {
    stopCamera();
  }

  try {
    const constraints = {
      video: deviceId ? { deviceId: { exact: deviceId } } : { facingMode: 'environment' },
      audio: false
    };

    cameraStream = await navigator.mediaDevices.getUserMedia(constraints);
    const track = cameraStream.getVideoTracks()[0];
    cameraDeviceId = track.getSettings().deviceId || deviceId || null;

    document.querySelectorAll('[data-camera-video]').forEach((video) => {
      video.srcObject = cameraStream;
      video.play().catch(() => {});
      video.classList.add('active');
    });

    document.querySelectorAll('.camera-placeholder, .live-camera-placeholder').forEach((el) => {
      el.style.display = 'none';
    });

    updateCameraStatus(true);
    populateCameraSelect();
    return cameraStream;
  } catch (error) {
    console.error('Unable to start camera:', error);
    if (deviceId) {
      // retry with default device if specific one failed
      return startCamera(null);
    }
    showCameraError(error.message || 'Could not access camera. Check permissions.');
    return null;
  }
}

function stopCamera() {
  if (cameraStream) {
    cameraStream.getTracks().forEach((track) => track.stop());
    cameraStream = null;
    document.querySelectorAll('[data-camera-video]').forEach((video) => {
      video.pause();
      video.srcObject = null;
      video.classList.remove('active');
    });
    document.querySelectorAll('.camera-placeholder, .live-camera-placeholder').forEach((el) => {
      el.style.display = 'grid';
    });
    updateCameraStatus(false);
  }
}

function initYoloStream() {
  const img = document.getElementById('yolo-stream');
  if (!img) return;
  img.onload = () => {
    img.style.display = 'block';
    const vid = document.getElementById('dashboard-video');
    if (vid) vid.style.display = 'none';
    document.querySelectorAll('.camera-placeholder').forEach((el) => { el.style.display = 'none'; });
    updateCameraStatus(true, 'YOLO stream');
  };
  img.onerror = () => {
    img.style.display = 'none';
    const vid = document.getElementById('dashboard-video');
    if (vid) {
      vid.style.display = 'block';
      startCamera();
    }
    document.querySelectorAll('.camera-placeholder').forEach((el) => { el.style.display = 'grid'; });
    updateCameraStatus(false, 'Stream unavailable');
  };
  img.src = YOLO_STREAM_URL + '?t=' + Date.now();
}

function setupCameraControls() {
  const select = document.getElementById('camera-select');
  const refreshBtn = document.getElementById('camera-refresh');
  const restartBtn = document.getElementById('camera-restart');

  if (select) {
    select.addEventListener('change', async (e) => {
      const newId = e.target.value;
      cameraDeviceId = newId || null;
      await startCamera(cameraDeviceId);
    });
  }

  if (refreshBtn) {
    refreshBtn.addEventListener('click', async () => {
      await listCameraDevices(true);
    });
  }

  if (restartBtn) {
    restartBtn.addEventListener('click', async () => {
      await startCamera(cameraDeviceId);
    });
  }
}

// Dashboard Maps
function initDashboardMaps() {
  if (typeof L === 'undefined') {
    console.error('Leaflet not loaded');
    return;
  }
  
  if (!locationMap) {
    const mapEl = document.getElementById('location-map');
    if (!mapEl) return;
    
    locationMap = L.map('location-map').setView([28.6129, 77.2295], 15);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19,
      attribution: '© OpenStreetMap'
    }).addTo(locationMap);
    
    locationMarker = L.marker([28.6129, 77.2295], {
      icon: L.divIcon({
        className: 'boat-marker',
        html: '<div style="background: #1f6bff; width: 24px; height: 24px; border-radius: 50%; border: 3px solid #fff; box-shadow: 0 2px 8px rgba(0,0,0,0.3);"></div>',
        iconSize: [24, 24]
      })
    }).addTo(locationMap);
    
    // Draw path
    const path = [
      [28.6120, 77.2280],
      [28.6125, 77.2290],
      [28.6129, 77.2295],
      [28.6135, 77.2300]
    ];
    L.polyline(path, { color: '#1f6bff', dashArray: '5, 5' }).addTo(locationMap);
  }
}

// Live Control Maps
function initLiveMaps() {
  if (typeof L === 'undefined') {
    console.error('Leaflet not loaded');
    return;
  }
  
  if (!gpsMap) {
    const mapEl = document.getElementById('gps-map');
    if (!mapEl) return;
    
    gpsMap = L.map('gps-map').setView([28.6129, 77.2295], 14);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19
    }).addTo(gpsMap);
    
    gpsMarker = L.marker([28.6129, 77.2295], {
      icon: L.divIcon({
        className: 'gps-marker',
        html: '<div style="background: #1f6bff; width: 32px; height: 32px; border-radius: 50%; border: 3px solid #fff; display: grid; place-items: center; color: #fff; font-weight: bold;">✈️</div>',
        iconSize: [32, 32]
      })
    }).addTo(gpsMap);
    
    // Draw path
    const gpsPath = [
      [28.6115, 77.2275],
      [28.6120, 77.2285],
      [28.6125, 77.2290],
      [28.6129, 77.2295],
      [28.6135, 77.2305]
    ];
    L.polyline(gpsPath, { color: '#1f6bff', dashArray: '10, 5' }).addTo(gpsMap);
  }
}

// Heatmap
function initHeatmap() {
  if (typeof L === 'undefined') {
    console.error('Leaflet not loaded');
    return;
  }
  
  if (!pollutionMap) {
    const mapEl = document.getElementById('pollution-map');
    if (!mapEl) return;
    
    pollutionMap = L.map('pollution-map').setView([28.6129, 77.2295], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19
    }).addTo(pollutionMap);
    
    // Add heatmap data
    const heatData = [
      [28.6129, 77.2295, 0.8],
      [28.6140, 77.2310, 0.9],
      [28.6115, 77.2280, 0.7],
      [28.6135, 77.2305, 0.85]
    ];
    
    // Try to add heat layer if available
    try {
      if (typeof L !== 'undefined' && typeof L.heatLayer !== 'undefined') {
        heatLayer = L.heatLayer(heatData, {
          radius: 50,
          blur: 30,
          maxZoom: 17,
          max: 1.0,
          gradient: { 0.4: 'blue', 0.6: 'cyan', 0.7: 'lime', 0.8: 'yellow', 1.0: 'red' }
        }).addTo(pollutionMap);
      } else {
        console.warn('Leaflet.heat plugin not loaded, using markers instead');
        // Fallback to regular markers
        heatData.forEach(([lat, lon, intensity]) => {
          const color = intensity > 0.8 ? '#ef4444' : intensity > 0.6 ? '#f59e0b' : '#10b981';
          L.circleMarker([lat, lon], {
            radius: intensity * 20,
            fillColor: color,
            color: '#fff',
            weight: 2,
            opacity: 0.8,
            fillOpacity: 0.6
          }).addTo(pollutionMap);
        });
      }
    } catch (error) {
      console.error('Error initializing heatmap:', error);
    }
  }
}

// Predictions Maps
function initPredictions() {
  if (typeof L === 'undefined') {
    console.error('Leaflet not loaded');
    return;
  }
  
  if (!zonesMap) {
    const mapEl = document.getElementById('zones-map');
    if (!mapEl) return;
    
    zonesMap = L.map('zones-map').setView([28.6129, 77.2295], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19
    }).addTo(zonesMap);
    
    // Add zone markers
    const zones = [
      { lat: 28.618, lon: 77.235, risk: 'high', name: 'Zone E' },
      { lat: 28.615, lon: 77.231, risk: 'high', name: 'Zone B' },
      { lat: 28.62, lon: 77.24, risk: 'high', name: 'Zone D' },
      { lat: 28.61, lon: 77.228, risk: 'medium', name: 'Zone A' },
      { lat: 28.613, lon: 77.232, risk: 'low', name: 'Zone C' }
    ];
    
    zones.forEach(zone => {
      const color = zone.risk === 'high' ? '#ef4444' : zone.risk === 'medium' ? '#fbbf24' : '#10b981';
      L.marker([zone.lat, zone.lon], {
        icon: L.divIcon({
          className: 'zone-marker',
          html: `<div style="background: ${color}; width: 20px; height: 20px; border-radius: 50%; border: 2px solid #fff; box-shadow: 0 2px 8px rgba(0,0,0,0.3);"></div>`,
          iconSize: [20, 20]
        })
      }).addTo(zonesMap).bindPopup(zone.name);
    });
  }
}

// Charts Initialization
function initCharts() {
  // Wait for Chart.js to be available
  if (typeof Chart === 'undefined') {
    console.error('Chart.js not loaded');
    return;
  }

  // Waste Collection Chart
  const wasteCtx = document.getElementById('wasteChart');
  if (wasteCtx && wasteCtx.getContext) {
    charts.waste = new Chart(wasteCtx, {
      type: 'bar',
      data: {
        labels: ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun'],
        datasets: [
          {
            label: 'Dry Waste',
            data: [8.2, 10.5, 8.8, 11.5, 13, 14.5, 12.5],
            backgroundColor: '#2563eb'
          },
          {
            label: 'Wet Waste',
            data: [4.8, 6.8, 5.5, 7.5, 8, 9.5, 7.8],
            backgroundColor: '#10b981'
          }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        plugins: {
          legend: {
            position: 'bottom'
          }
        },
        scales: {
          y: {
            beginAtZero: true,
            title: {
              display: true,
              text: 'Weight (Kg)'
            }
          }
        }
      }
    });
  }

  // TDS Trend
  const tdsCtx = document.getElementById('tdsTrend');
  if (tdsCtx && tdsCtx.getContext) {
    charts.tds = new Chart(tdsCtx, {
      type: 'line',
      data: {
        labels: ['00:00', '04:00', '08:00', '12:00', '16:00', '20:00', '23:59'],
        datasets: [{
          label: 'TDS (ppm)',
          data: [295, 290, 305, 320, 330, 325, 320],
          borderColor: '#2563eb',
          backgroundColor: 'rgba(37, 99, 235, 0.1)',
          tension: 0.4,
          fill: true
        }]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        plugins: {
          legend: {
            display: false
          }
        },
        scales: {
          y: {
            beginAtZero: false,
            min: 190,
            max: 380
          }
        }
      }
    });
  }

  // Temperature Trend
  const tempCtx = document.getElementById('tempTrend');
  if (tempCtx && tempCtx.getContext) {
    charts.temp = new Chart(tempCtx, {
      type: 'line',
      data: {
        labels: ['00:00', '04:00', '08:00', '12:00', '16:00', '20:00', '23:59'],
        datasets: [{
          label: 'Temperature (°C)',
          data: [27, 26, 28, 30, 31, 29.5, 28],
          borderColor: '#f97316',
          backgroundColor: 'rgba(249, 115, 22, 0.1)',
          tension: 0.4,
          fill: true
        }]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        plugins: {
          legend: {
            display: false
          }
        },
        scales: {
          y: {
            beginAtZero: false,
            min: 16,
            max: 32
          }
        }
      }
    });
  }

  // Air Quality Trend
  const aqiCtx = document.getElementById('aqiTrend');
  if (aqiCtx && aqiCtx.getContext) {
    charts.aqi = new Chart(aqiCtx, {
      type: 'line',
      data: {
        labels: ['00:00', '04:00', '08:00', '12:00', '16:00', '20:00', '23:59'],
        datasets: [{
          label: 'Air Quality (PPM)',
          data: [145, 148, 152, 165, 175, 170, 162],
          borderColor: '#22c55e',
          backgroundColor: 'rgba(34, 197, 94, 0.2)',
          tension: 0.4,
          fill: true
        }]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        plugins: {
          legend: {
            display: false
          }
        },
        scales: {
          y: {
            beginAtZero: false,
            min: 150,
            max: 200
          }
        }
      }
    });
  }

  // Correlation Chart
  const corrCtx = document.getElementById('correlationChart');
  if (corrCtx && corrCtx.getContext) {
    charts.correlation = new Chart(corrCtx, {
      type: 'bar',
      data: {
        labels: ['Zone A', 'Zone B', 'Zone C', 'Zone D', 'Zone E'],
        datasets: [
          {
            label: 'TDS Level',
            data: [375, 400, 300, 375, 450],
            backgroundColor: '#2563eb',
            yAxisID: 'y'
          },
          {
            label: 'Waste Collected',
            data: [15, 20, 12, 18, 25],
            backgroundColor: '#10b981',
            yAxisID: 'y1'
          }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        plugins: {
          legend: {
            position: 'bottom'
          }
        },
        scales: {
          y: {
            type: 'linear',
            display: true,
            position: 'left',
            title: {
              display: true,
              text: 'TDS (ppm)'
            },
            min: 0,
            max: 600
          },
          y1: {
            type: 'linear',
            display: true,
            position: 'right',
            title: {
              display: true,
              text: 'Waste (Kg)'
            },
            min: 0,
            max: 28,
            grid: {
              drawOnChartArea: false
            }
          }
        }
      }
    });
  }

  // Forecast Chart
  const forecastCtx = document.getElementById('forecastChart');
  if (forecastCtx && forecastCtx.getContext) {
    charts.forecast = new Chart(forecastCtx, {
      type: 'line',
      data: {
        labels: ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun'],
        datasets: [
          {
            label: 'Actual',
            data: [13, 15, 17, 18.5, 20, null, null],
            borderColor: '#2563eb',
            backgroundColor: 'rgba(37, 99, 235, 0.1)',
            tension: 0.4,
            borderWidth: 2
          },
          {
            label: 'Forecast',
            data: [null, null, null, null, null, 21, 22],
            borderColor: '#8b5cf6',
            backgroundColor: 'rgba(139, 92, 246, 0.1)',
            tension: 0.4,
            borderWidth: 2,
            borderDash: [5, 5]
          }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        plugins: {
          legend: {
            position: 'bottom'
          }
        },
        scales: {
          y: {
            beginAtZero: true,
            title: {
              display: true,
              text: 'Waste (Kg)'
            }
          }
        }
      }
    });
  }

  // Pollution Forecast
  const pollutionCtx = document.getElementById('pollutionForecast');
  if (pollutionCtx && pollutionCtx.getContext) {
    charts.pollution = new Chart(pollutionCtx, {
      type: 'line',
      data: {
        labels: ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun'],
        datasets: [
          {
            label: 'Air Quality (PPM)',
            data: [150, 155, 160, 170, 175, 180, 190],
            borderColor: '#22c55e',
            backgroundColor: 'rgba(34, 197, 94, 0.3)',
            tension: 0.4,
            fill: true
          },
          {
            label: 'TDS (ppm)',
            data: [350, 355, 370, 380, 390, 400, 405],
            borderColor: '#2563eb',
            backgroundColor: 'rgba(37, 99, 235, 0.3)',
            tension: 0.4,
            fill: true
          }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        plugins: {
          legend: {
            position: 'bottom'
          }
        },
        scales: {
          y: {
            beginAtZero: false,
            min: 0,
            max: 400
          }
        }
      }
    });
  }
}

// Update Gauges
function updateGauges(dryPercent, wetPercent) {
  const dryGauge = document.getElementById('dry-gauge');
  const wetGauge = document.getElementById('wet-gauge');
  
  if (dryGauge) {
    const dryValue = document.querySelector('#dry-gauge .gauge-value');
    if (dryValue) dryValue.textContent = `${dryPercent}%`;
    dryGauge.style.background = `conic-gradient(from 180deg, #2563eb 0%, #2563eb ${dryPercent}%, #e5e7eb ${dryPercent}%, #e5e7eb 100%)`;
  }
  
  if (wetGauge) {
    const wetValue = document.querySelector('#wet-gauge .gauge-value');
    if (wetValue) wetValue.textContent = `${wetPercent}%`;
    wetGauge.style.background = `conic-gradient(from 180deg, #10b981 0%, #10b981 ${wetPercent}%, #e5e7eb ${wetPercent}%, #e5e7eb 100%)`;
  }
}

// Update Telemetry
function updateTelemetry(data) {
  // Update header battery
  const headerBattery = document.getElementById('header-battery');
  if (headerBattery && data.battery_volt) {
    const percent = Math.round((data.battery_volt / 12.6) * 100);
    headerBattery.textContent = `${Math.min(100, percent)}%`;
  }

  // Update status battery
  const statusBattery = document.getElementById('status-battery');
  if (statusBattery && data.battery_volt) {
    const percent = Math.round((data.battery_volt / 12.6) * 100);
    statusBattery.textContent = `${Math.min(100, percent)}%`;
  }

  // Update camera GPS
  const cameraGPS = document.getElementById('camera-gps');
  if (cameraGPS && data.gps) {
    cameraGPS.textContent = `${data.gps.lat.toFixed(4)}° N, ${data.gps.lon.toFixed(4)}° E`;
  }

  // Update camera speed
  const cameraSpeed = document.getElementById('camera-speed');
  if (cameraSpeed && data.speed) {
    cameraSpeed.textContent = (data.speed * 3.6).toFixed(1); // m/s to km/h
  }

  // Update location
  const locationLat = document.getElementById('location-lat');
  if (locationLat && data.gps) {
    locationLat.textContent = `${data.gps.lat.toFixed(2)}°`;
  }

  // Update environmental metrics
  const envTDS = document.getElementById('env-tds');
  if (envTDS && data.tds_ppm) {
    envTDS.textContent = `${data.tds_ppm} ppm`;
  }

  const envTemp = document.getElementById('env-temp');
  if (envTemp && data.temperature_c) {
    envTemp.textContent = `${data.temperature_c.toFixed(1)} °C`;
  }

  const envAQI = document.getElementById('env-aqi');
  if (envAQI && data.mq135_ppm) {
    envAQI.textContent = `${data.mq135_ppm} PPM`;
  }

  // Update waste gauges
  if (data.waste) {
    const total = (data.waste.dry || 0) + (data.waste.wet || 0);
    if (total > 0) {
      const dryPercent = Math.round((data.waste.dry / total) * 100);
      const wetPercent = Math.round((data.waste.wet / total) * 100);
      updateGauges(dryPercent, wetPercent);
    }
  }

  // Update maps
  if (data.gps && locationMarker) {
    locationMarker.setLatLng([data.gps.lat, data.gps.lon]);
    locationMap.setView([data.gps.lat, data.gps.lon]);
  }

  if (data.gps && gpsMarker) {
    gpsMarker.setLatLng([data.gps.lat, data.gps.lon]);
    gpsMap.setView([data.gps.lat, data.gps.lon]);
    
    // Update tooltip
    const tooltipLat = document.getElementById('tooltip-lat');
    const tooltipLon = document.getElementById('tooltip-lon');
    const tooltipHeading = document.getElementById('tooltip-heading');
    
    if (tooltipLat) tooltipLat.textContent = `${data.gps.lat.toFixed(4)}°`;
    if (tooltipLon) tooltipLon.textContent = `${data.gps.lon.toFixed(4)}°`;
    if (tooltipHeading && data.heading) {
      tooltipHeading.textContent = `${data.heading.toString().padStart(3, '0')}°`;
    }
  }
}

// Controls
function initControls() {
  // Speed slider
  const speedSlider = document.getElementById('speed-slider');
  const speedValue = document.getElementById('speed-value');
  if (speedSlider && speedValue) {
    speedSlider.addEventListener('input', (e) => {
      speedValue.textContent = `${e.target.value}%`;
      sendCommand({ motor_speed: parseInt(e.target.value) });
    });
  }

  // D-pad controls
  document.querySelectorAll('.dpad-btn').forEach(btn => {
    btn.addEventListener('click', () => {
      const cmd = btn.dataset.cmd;
      if (cmd) {
        sendCommand({ action: cmd });
      }
    });
  });

  // Quick actions
  document.querySelectorAll('.btn-quick').forEach(btn => {
    btn.addEventListener('click', () => {
      const text = btn.textContent.trim();
      if (text.includes('Home')) {
        sendCommand({ action: 'return_home' });
      } else if (text.includes('Collection')) {
        sendCommand({ action: 'start_collection' });
      } else if (text.includes('Flap')) {
        sendCommand({ action: 'open_flap' });
      }
    });
  });

  // Autonomous toggle
  const autonomousToggle = document.getElementById('autonomous-toggle');
  if (autonomousToggle) {
    autonomousToggle.addEventListener('change', (e) => {
      sendCommand({ mode: e.target.checked ? 'autonomous' : 'manual' });
    });
  }

  // Layer buttons
  document.querySelectorAll('.layer-btn').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.layer-btn').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      const layer = btn.dataset.layer;
      // Update heatmap layer
      updateHeatmapLayer(layer);
    });
  });

  // Period options
  document.querySelectorAll('.period-option').forEach(option => {
    option.addEventListener('click', () => {
      document.querySelectorAll('.period-option').forEach(o => o.classList.remove('active'));
      option.classList.add('active');
    });
  });

  // Export buttons
  document.querySelectorAll('.btn-export-csv, .btn-export-pdf, .btn-export-table').forEach(btn => {
    btn.addEventListener('click', () => {
      const type = btn.classList.contains('btn-export-csv') ? 'csv' :
                   btn.classList.contains('btn-export-pdf') ? 'pdf' : 'csv';
      downloadFile(`/api/export/${type}`);
    });
  });

  // Camera start/stop buttons
  const startBtn = document.getElementById('camera-start-btn');
  const stopBtn = document.getElementById('camera-stop-btn');
  if (startBtn) {
    startBtn.addEventListener('click', () => startCamera());
  }
  if (stopBtn) {
    stopBtn.addEventListener('click', () => stopCamera());
  }

  // Start backend button
  const startBackendBtn = document.getElementById('start-backend-btn');
  if (startBackendBtn) {
    startBackendBtn.addEventListener('click', async () => {
      startBackendBtn.disabled = true;
      startBackendBtn.textContent = 'Starting...';
      try {
        const res = await fetch(`${API_BASE}/api/control/start-backend`, { method: 'POST' });
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        startBackendBtn.textContent = 'Backend Started';
      } catch (err) {
        console.error('Failed to start backend', err);
        startBackendBtn.textContent = 'Retry Start Backend';
      } finally {
        setTimeout(() => {
          startBackendBtn.disabled = false;
          startBackendBtn.textContent = '🖥️ Start Backend';
        }, 2000);
      }
    });
  }
}

function updateHeatmapLayer(layer) {
  // Update heatmap based on selected layer
  console.log('Switching to layer:', layer);
}

// Commands
function sendCommand(cmd) {
  fetch(`${API_BASE}/api/control/send`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(cmd)
  }).catch(console.error);
  
  if (socket) {
    socket.emit('command', cmd);
  }
}

// Data Loading
async function fetchJson(path) {
  try {
    const res = await fetch(`${API_BASE}${path}`);
    return await res.json();
  } catch (error) {
    console.error('Fetch error:', error);
    return { data: [] };
  }
}

async function loadRecords() {
  const data = await fetchJson('/api/analytics/weekly');
  const tbody = document.getElementById('records-tbody');
  if (!tbody) return;

  // Generate sample records for demo
  const records = [
    { date: '2025-11-16', dry: 12.4, wet: 7.8, tds: 342, temp: 28.5, aqi: 156, gps: '28.6129°N, 77.2295°E', status: 'Completed' },
    { date: '2025-11-15', dry: 15.8, wet: 9.5, tds: 355, temp: 29.1, aqi: 168, gps: '28.6142°N, 77.2308°E', status: 'Completed' },
    { date: '2025-11-14', dry: 13.2, wet: 8.1, tds: 365, temp: 29.5, aqi: 185, gps: '28.6135°N, 77.2315°E', status: 'Completed' },
    { date: '2025-11-13', dry: 11.5, wet: 7.3, tds: 340, temp: 28.2, aqi: 172, gps: '28.6121°N, 77.2289°E', status: 'Completed' },
    { date: '2025-11-12', dry: 9.1, wet: 5.9, tds: 315, temp: 27.8, aqi: 145, gps: '28.6118°N, 77.2275°E', status: 'Completed' },
    { date: '2025-11-11', dry: 10.2, wet: 6.8, tds: 328, temp: 28, aqi: 152, gps: '28.6125°N, 77.2282°E', status: 'Completed' },
    { date: '2025-11-10', dry: 8.5, wet: 5.2, tds: 305, temp: 27.5, aqi: 138, gps: '28.6112°N, 77.2268°E', status: 'Completed' }
  ];

  tbody.innerHTML = records.map(rec => {
    const tdsClass = rec.tds > 350 ? 'highlight-red' : '';
    const aqiClass = rec.aqi > 170 ? 'highlight-orange' : '';
    return `
      <tr>
        <td>${rec.date}</td>
        <td>${rec.dry}</td>
        <td>${rec.wet}</td>
        <td class="${tdsClass}">${rec.tds}</td>
        <td>${rec.temp}</td>
        <td class="${aqiClass}">${rec.aqi}</td>
        <td>${rec.gps}</td>
        <td><span class="status-badge">${rec.status}</span></td>
      </tr>
    `;
  }).join('');
}

function downloadFile(path) {
  window.open(`${API_BASE}${path}`, '_blank');
}

// WebSocket Connection
function initWebSocket() {
  socket = io(API_BASE, { transports: ['websocket'] });
  
  socket.on('connect', () => {
    console.log('WebSocket connected');
  });
  
  socket.on('telemetry', (payload) => {
    const data = payload.data || {};
    updateTelemetry(data);
  });
  
  socket.on('disconnect', () => {
    console.log('WebSocket disconnected');
  });
}

// Bootstrap
async function bootstrap() {
  initNavigation();
  initCharts();
  initControls();
  initDashboardMaps();
  setupCameraControls();
  await listCameraDevices(true);
  await startCamera(cameraDeviceId);
  initYoloStream();
  await loadRecords();
  initWebSocket();
  
  // Initial data load
  try {
    const latest = await fetchJson('/api/sensors/latest?n=1');
    if (latest.data && latest.data[0]) {
      updateTelemetry(latest.data[0].data || {});
    }
  } catch (error) {
    console.error('Initial data load error:', error);
  }
  
  // Set initial gauge values
  updateGauges(68, 32);
}

// Initialize on DOM ready
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', bootstrap);
} else {
  bootstrap();
}
