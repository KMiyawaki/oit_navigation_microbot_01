const OitJs = {};

OitJs.getRosTimeStamp = function () {
  const currentTime = new Date();
  const secs = Math.floor(currentTime.getTime() / 1000);
  const nsecs = Math.round(1000000000 * (currentTime.getTime() / 1000 - secs));
  return { secs: secs, nsecs: nsecs };
}

OitJs.toRadian = function (degree) {
  return degree * Math.PI / 180;
}

OitJs.toDegree = function (radian) {
  return radian / Math.PI * 180;
}
OitJs.SimplePose2D = class {
  constructor(x, y, theta) {
    this.x = x;
    this.y = y;
    this.theta = theta;
  }

  toString(precision = 2) {
    return "(" + this.x.toFixed(precision) + "," + this.y.toFixed(precision) + ") " + OitJs.toDegree(this.theta).toFixed(precision) + "(deg)";
  }

  to3D(z = 0) {
    const q = new THREE.Quaternion();
    q.setFromEuler(new THREE.Euler(0, 0, this.theta));
    let pose = {
      position: { x: this.x, y: this.y, z: z },
      orientation: { x: q.x, y: q.y, z: q.z, w: q.w }
    };
    return pose;
  }
}

OitJs.buildPoseWithCovarianceStamped = function (frameId, pose3D, varX, varY, varTheta) {
  let message = new ROSLIB.Message({
    header: {
      seq: 0,
      stamp: OitJs.getRosTimeStamp(),
      frame_id: frameId
    },
    pose: {
      pose: pose3D,
      covariance: [
        varX, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, varY, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, varTheta]
    }
  });
  return message;
}

OitJs.tf2Pose2D = function (tf) {
  const x = tf.translation.x;
  const y = tf.translation.y;
  const q = new THREE.Quaternion(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w);
  const e = new THREE.Euler();
  e.setFromQuaternion(q);
  return new OitJs.SimplePose2D(x, y, e.z);
}

OitJs.saveTxt = function (filename, data) {
  const blob = new Blob([data], { type: 'text/plain' });
  if (window.navigator.msSaveOrOpenBlob) {
    window.navigator.msSaveBlob(blob, filename);
  }
  else {
    let elem = window.document.createElement('a');
    elem.href = window.URL.createObjectURL(blob);
    elem.download = filename;
    document.body.appendChild(elem);
    elem.click();
    document.body.removeChild(elem);
  }
}

OitJs.mapPgm = function (occupancyGridMsg, thresholdOccupied = 65, thresholdFree = 25) {
  // https://github.com/ros-planning/navigation/blob/melodic-devel/map_server/src/map_saver.cpp
  const metadata = occupancyGridMsg.info;
  const data = occupancyGridMsg.data;
  let header = "";
  const encoder = new TextEncoder();

  // Write header with plain text.
  header += "P5\n";
  header += "# CREATOR: oit.js " + metadata.resolution.toFixed(2) + " m/pix\n";
  header += (metadata.width + " " + metadata.height + "\n255\n");
  const tmpArray = encoder.encode(header);
  let targetIdx = tmpArray.length;
  const pgm = new Uint8Array(targetIdx + metadata.width * metadata.height);
  pgm.set(tmpArray);

  // Write pixel data with binary data.
  for (let y = 0; y< metadata.height; y++) {
    for (let x = 0; x < metadata.width; x++) {
      const i = x + (metadata.height - y - 1) * metadata.width;
      let value = 205; //occ [0.25,0.65]
      if (data[i] >= 0 && data[i] <= thresholdFree) { // [0,free)
        value = 254;
      } else if (data[i] >= thresholdOccupied) { // (occ,255]
        value = 0;
      }
      pgm[targetIdx++] = value;
    }
  }
  return pgm;
}

OitJs.mapMetaDataYaml = function (metadata, map_name, precision = 6) {
  // http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
  const q = new THREE.Quaternion(metadata.origin.orientation);
  const e = new THREE.Euler();
  e.setFromQuaternion(q, 'XYZ');
  let yaml = "";
  yaml += "image: " + map_name + ".pgm\n";
  yaml += "resolution: " + metadata.resolution.toFixed(precision) + "\n";
  yaml += "origin: [";
  yaml += metadata.origin.position.x.toFixed(precision) + ", ";
  yaml += metadata.origin.position.y.toFixed(precision) + ", ";
  yaml += e.z.toFixed(precision);
  yaml += "]\n";
  yaml += "negate: 0\n";
  yaml += "occupied_thresh: 0.65\n";
  yaml += "free_thresh: 0.196\n";
  return yaml;
}

OitJs.getCameraUpDirectionInWorld = function (camera) {
  let upV = new THREE.Vector3(0, 1, 0);
  upV.applyQuaternion(camera.quaternion);
  return upV;
}

OitJs.vector3Str = function (v, precision = 2) {
  return v.x.toFixed(precision) + "," + v.y.toFixed(precision) + "," + v.z.toFixed(precision);
}

OitJs.wrap2Pi = function (radian) {
  while (Math.abs(radian) > Math.PI) {
    if (radian < 0) {
      radian += Math.PI * 2;
    } else {
      radian -= Math.PI * 2;
    }
  }
  return radian;
}

OitJs.joyStickStr = function (joyStickVector, precision = 2) {
  return ("Joypad:" + joyStickVector.x.toFixed(precision) + "," + joyStickVector.y.toFixed(precision));
}

OitJs.WebVideoServerURL = class {
  constructor(host, topic, width = 1280/2.5, height = 720/2.5, quality = 40, port = 8080) {
    this._host = host;
    this._topic = topic;
    this._width = width;
    this._height = height;
    this._port = port;
    this._quality = quality;
  }

  getTopicName() {
    return this._topic;
  }

  get() {
    return "http://" + this._host + ":" + this._port + "/stream?topic=" + this._topic + "&width=" + this._width + "&height=" + this._height + "&quality=" + this._quality;
  }

  toHtml() {
    let html = "<b>web_video_server config</b><br />";
    html += "host:" + this._host + " ";    
    html += "port:" + this._port + "<br />";
    html += "topic:" + this._topic + "<br />";
    html += "width:" + this._width + ", height:" + this._height + " ";
    html += "quality:" + this._quality;
    return html;
  }
}

OitJs.ImageTopicSelector = class {
  constructor(hostName, topics, selectBox, imgElement, configDisplay){
    this._videoServers = {};
    for (let i = 0;i <  topics.length; i++) {
      const topic = topics[i];
      const url = new OitJs.WebVideoServerURL(hostName, topic);
      const opt = document.createElement('option');
      opt.value = topic;
      opt.innerHTML = topic;
      selectBox.appendChild(opt);
      this._videoServers[topic] = url;
    }
    this._selectBox = selectBox;
    this._imgElement = imgElement;
    this._configDisplay = configDisplay;
  }

  setImageURL (idx) {
    const value = this._selectBox.options[idx].value;
    this._imgElement.src = this._videoServers[value].get();
    this._configDisplay.innerHTML = this._videoServers[value].toHtml();
  }

  init(){
    this.setImageURL(0);
    const me = this;
    const select = this._selectBox;
    select.onchange = function () {
      me.setImageURL(select.selectedIndex);
    }
  }
}

OitJs.WorldDirectionFromView = class {
  constructor(camera) {
    this._camera = camera;
  }

  getDirection(rotateZ, length = 0.1) {
    let upV = OitJs.getCameraUpDirectionInWorld(this._camera);
    upV.z = 0;
    upV.normalize();
    upV.multiplyScalar(length);
    upV.applyEuler(new THREE.Euler(0, 0, rotateZ));
    return upV;
  }

  getDirectionFromJoy(vector2D, length = 0.1) {
    return this.getDirection(Math.atan2(vector2D.y, vector2D.x), length);
  }

  moveTo(position, rotateZ, length = 0.1) {
    let d = this.getDirection(rotateZ, length);
    position.x += d.x;
    position.y += d.y;
  }

  moveToByJoy(position, vector2D, ratio = 0.1, max = 0.3, min = 0.01) {
    let mag = Math.hypot(vector2D.x * ratio, vector2D.y * ratio);
    mag = Math.min(mag, max);
    if (mag > min) {
      this.moveTo(position, Math.atan2(vector2D.y, vector2D.x), mag);
    }
  }

  moveForward(position, length = 0.1) {
    this.moveTo(position, 0, length);
  }

  moveBackward(position, length = 0.1) {
    this.moveTo(position, OitJs.toRadian(180), length);
  }

  moveLeft(position, length = 0.1) {
    this.moveTo(position, OitJs.toRadian(90), length);
  }

  moveRight(position, length = 0.1) {
    this.moveTo(position, OitJs.toRadian(-90), length);
  }

  getForward(length = 0.1) {
    return this.getDirection(0, length);
  }

  getBackward(length = 0.1) {
    return this.getDirection(OitJs.toRadian(180), length);
  }

  getLeft(length = 0.1) {
    return this.getDirection(OitJs.toRadian(90), length);
  }

  getRight(length = 0.1) {
    return this.getDirection(OitJs.toRadian(-90), length);
  }
}

OitJs.JoyStick = class {
  constructor(container, width, height, verbose = false) {
    this._verbose = verbose;
    this._stickPos = null;
    this._baseColorStopping = "cadetblue";
    this._baseColorMoving = "orangered";
    this._strokeStyle = 'cyan';
    this._container = container;
    this._container.style.width = width + 'px';
    this._container.style.height = height + 'px';
    this._container.style.background = this._baseColorStopping;
    this._supportTouch = 'ontouchend' in document;
    this._canvas = document.createElement('canvas');
    const rect = this._container.getBoundingClientRect();
    this._canvas.width = rect.width;
    this._canvas.height = rect.height;
    this._container.appendChild(this._canvas);
    this._center = { x: this._canvas.width / 2, y: this._canvas.height / 2 };
    this._radius = Math.min(rect.width / 2, rect.height / 2);
    this.update();
    /*http://xoinu.hatenablog.com/entry/20061025/1161843622*/
    const obj = this;
    console.log("Touch support  = " + this._supportTouch);
    if (this._supportTouch) {
      this._container.addEventListener('touchstart', function (evt) { obj.onTouchStart(evt); });
      this._container.addEventListener('touchmove', function (evt) { obj.onTouchMove(evt); });
      this._container.addEventListener('touchend', function (evt) { obj.onTouchEnd(evt); });
    } else {
      this._container.addEventListener('mousedown', function (evt) { obj.onMouseDown(evt); });
      this._container.addEventListener('mousemove', function (evt) { obj.onMouseMove(evt); });
      this._container.addEventListener('mouseup', function (evt) { obj.onMouseUp(evt); });
      this._container.addEventListener('mouseleave', function (evt) { obj.onMouseLeave(evt); });
    }
  }

  outputLog(arg) {
    if (this._verbose) {
      console.log(arg);
    }
  }

  isMoving() {
    return (this._stickPos !== null && this._stickPos !== undefined);
  }

  cutOff(value, absMax = 1.0) {
    const absVal = Math.min(Math.abs(value), absMax);
    if (value < 0) {
      return -absVal;
    }
    return absVal;
  }

  getStickVector() {
    let vect = { x: 0.0, y: 0.0 };
    if (this.isMoving()) {
      const d = { x: this._stickPos.x - this._center.x, y: this._stickPos.y - this._center.y };
      vect.x = this.cutOff(-d.y / this._radius);
      vect.y = this.cutOff(-d.x / this._radius);
    }
    return vect;
  }

  update() {
    const w = this._canvas.width;
    const h = this._canvas.height;
    const cx = w / 2;
    const cy = h / 2;
    const ctx = this._canvas.getContext('2d');
    ctx.clearRect(0, 0, w, h);
    ctx.beginPath();
    ctx.strokeStyle = this._strokeStyle;
    ctx.lineWidth = 6;
    ctx.arc(cx, cy, 40, 0, Math.PI * 2, true);
    ctx.stroke();

    ctx.beginPath();
    ctx.strokeStyle = this._strokeStyle;
    ctx.lineWidth = 2;
    ctx.arc(cx, cy, 60, 0, Math.PI * 2, true);
    ctx.stroke();

    if (this.isMoving()) {
      this._container.style.background = this._baseColorMoving;
      ctx.beginPath();
      ctx.strokeStyle = this._strokeStyle;
      ctx.lineWidth = 6;
      ctx.arc(this._stickPos.x, this._stickPos.y, 40, 0, Math.PI * 2, true);
      ctx.stroke();
    } else {
      this._container.style.background = this._baseColorStopping;
    }
  }

  calcStickPos(mouseX, mouseY) {
    const rect = this._container.getBoundingClientRect();
    this._stickPos = { x: mouseX - rect.x, y: mouseY - rect.y };
  }
  // For Mobile Phone
  onTouchStart(e) {
    e.preventDefault();
    const touch = e.touches[0];
    this.calcStickPos(touch.clientX, touch.clientY);
    this.update();
    this.outputLog('onTouchStart:' + this._stickPos);
  }

  onTouchMove(e) {
    e.preventDefault();
    this.outputLog('onTouchMove:');
    const touch = e.touches[0];
    this.calcStickPos(touch.clientX, touch.clientY);
    this.update();
    this.outputLog(this.getStickVector());
  }

  onTouchEnd(e) {
    e.preventDefault();
    this.outputLog('onTouchEnd:');
    this._stickPos = null;
    this.update();
  }
  // For PC
  onMouseDown(e) {
    this.calcStickPos(e.clientX, e.clientY);
    this.update();
    this.outputLog('onMouseDown:' + this._stickPos);
  }

  onMouseMove(e) {
    this.outputLog('onMouseMove:');
    if (this.isMoving()) {
      this.calcStickPos(e.clientX, e.clientY);
      this.update();
    }
    this.outputLog(this.getStickVector());
  }

  onMouseUp() {
    this.outputLog('onMouseUp:');
    this._stickPos = null;
    this.update();
  }

  onMouseLeave() {
    this.outputLog('onMouseLeave:');
    this._stickPos = null;
    this.update();
  }
}

OitJs.JoyStickToDiffDrive = {};

OitJs.JoyStickToDiffDrive.getTwist = function (joyStickVector, linearMax, angularMax) {
  linear = joyStickVector.x * linearMax;
  angular = joyStickVector.y * angularMax;
  const twist = new ROSLIB.Message({
    linear: {
      x: linear,
      y: 0,
      z: 0
    },
    angular: {
      x: 0,
      y: 0,
      z: angular
    }
  });
  return twist;
}

OitJs.JoyStickToDiffDrive.twistStr = function (twist, precision = 2) {
  return ("Linear:" + twist.linear.x.toFixed(precision) + ", Angular:" + OitJs.toDegree(twist.angular.z).toFixed(precision));
}
