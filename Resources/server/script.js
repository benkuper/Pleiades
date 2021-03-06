// LPS Head
// https://casual-effects.com/data/
// Triangles: 9923
// Vertices: 17684
// Updated: 2012-02-23
// License: CC BY 3.0
// © I-R Entertainment Ltd.

import { OrbitControls } from "https://unpkg.com/three@0.120.0/examples/jsm/controls/OrbitControls";
import * as EssentialsPlugin from "https://cdn.skypack.dev/@tweakpane/plugin-essentials@0.1.2";

//OBJECT
class PObject extends THREE.Group
{

  constructor(oid, type, data) {
    super();
    this.objectID = oid;
    this.type = type;

    this.color = new THREE.Color();
    this.color.setHSL(this.objectID / 10.0, 1, .5);
    this.ghostColor = new THREE.Color(.2,.2,.2);

    if(type == 0 || type == 1) this.setupCloud();
    if(type == 1)
    {
      this.setupBox(data);
      this.setupCentroid(data);
      this.setupVelocity(data);
    }

    this.update(data);
    
  }

  setupCloud()
  {

    this.cloudGeometry = new THREE.BufferGeometry();
    this.cloudMaterial = new THREE.PointsMaterial({
        size: 0.005,
        color: this.color,
        transparent: true,
        opacity: 0.8
      });

   this.cloudMesh =  new THREE.Points(this.cloudGeometry, this.cloudMaterial);
   this.add(this.cloudMesh);
  }

  setupBox()
  {
    this.boxGeometry = new THREE.BoxGeometry();
    this.boxMaterial = new THREE.MeshBasicMaterial({wireframe:true});
    this.boxMaterial.color = new THREE.Color();
    this.boxMesh = new THREE.Mesh( this.boxGeometry, this.boxMaterial );
    this.add(this.boxMesh);
  }
 
  setupCentroid()
  {
    this.centroidGeometry = new THREE.SphereGeometry();
    this.centroidMaterial = new THREE.MeshBasicMaterial();
    this.centroidMesh = new THREE.Mesh( this.centroidGeometry, this.centroidMaterial );
    this.add(this.centroidMesh);
  }

  setupVelocity()
  {
    this.velGeometry = new THREE.BufferGeometry();
    this.velMaterial = new THREE.LineBasicMaterial();
    this.velMesh = new THREE.LineSegments( this.velGeometry, this.velMaterial );
    this.add(this.velMesh);
  }


  update(data)
  {

    var deltaTime = (Date.now() - this.lastUpdateTime)/1000.0;

    var verticesIndex = 5;

    if(this.type == 1) //cluster
    {
      this.state = new Int32Array(data.slice(5,9))[0];
      var clusterData = new Float32Array(data.slice(9,9 + 12*4));

      var centroid = new THREE.Vector3(clusterData[0],clusterData[1],clusterData[2]);
      var velocity = new THREE.Vector3(clusterData[3],clusterData[4],clusterData[5]);
      var boxMin = new THREE.Vector3(clusterData[6],clusterData[7],clusterData[8]);
      var boxMax = new THREE.Vector3(clusterData[9],clusterData[10],clusterData[11]);

      var boxSize = new THREE.Vector3().add(boxMax).sub(boxMin);
      var boxCenter = new THREE.Vector3().add(boxMin).add(boxMax).divideScalar(2);
      
      this.boxGeometry =  new THREE.BoxGeometry(boxSize.x,boxSize.y,boxSize.z);
      this.boxGeometry.translate(boxCenter.x, boxCenter.y, boxCenter.z);
      this.boxMesh.geometry = this.boxGeometry;
      
      this.centroidGeometry =  new THREE.SphereGeometry(.01,32,16);
      this.centroidGeometry.translate(centroid.x, centroid.y, centroid.z);
      this.centroidMesh.geometry = this.centroidGeometry;

      const velPoints = [
        centroid,
        new THREE.Vector3().add(velocity).multiplyScalar(deltaTime).add(centroid)
       ];

      this.velGeometry = new THREE.BufferGeometry().setFromPoints(velPoints);
      this.velMesh.geometry = this.velGeometry;
    
      if(this.state == 3)
      {
        this.boxMaterial.color = this.ghostColor;
        this.centroidMaterial.color = this.ghostColor;
        this.velMaterial.color = this.ghostColor;
      } 
      else 
      {
        var bColor = new THREE.Color(this.color);
        bColor.offsetHSL(0,0,.2);
        this.boxMaterial.color = bColor;
        this.centroidMaterial.color = bColor;
        this.velMaterial.color = bColor;
      }

      verticesIndex += 4 + 12*4; //state = 4 bytes, boxMinMax = 6 * 4 bytes
    }

    if(this.state == 3)  this.cloudMaterial.color = this.ghostColor;
    else  this.cloudMaterial.color = this.color;

    var vertices = new Float32Array(data.slice(verticesIndex));
    this.cloudGeometry.setAttribute("position", new THREE.BufferAttribute(vertices, 3));
 
    this.lastUpdateTime = Date.now();
  }

  resetColors()
  {

  }
}


//APP

class App {
  constructor() {
 

    this.width = window.innerWidth;
    this.height = window.innerHeight;

    this.init();
    
    document.body.appendChild(this.renderer.domElement);
    
    this.resize();
    this.render();
    window.addEventListener("resize", this.resize.bind(this));
  }

  init() {

    //init scene
    this.scene = new THREE.Scene();

    //init renderer
    this.renderer = new THREE.WebGLRenderer();
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setSize(this.width, this.height);
    this.renderer.setClearColor(0xff222222, 1);
    this.renderer.physicallyCorrectLights = true
    this.renderer.outputEncoding = THREE.sRGBEncoding;

    //init camera
    this.camera = new THREE.PerspectiveCamera(
      70,
      this.width / this.height,
      0.001,
      1000
    );
    this.camera.position.set(-.5,1,-1.5);

    //init orbit
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    
    //init grid
    this.gridHelper = new THREE.GridHelper(1, 10);
    this.gridHelper.material.color.setHSL(0,0,.2);
    this.scene.add(this.gridHelper);


    //init objects
    this.objects = [];


    //init websocket
    this.connectWebSocket();

  }


  resize() {
    this.width = window.innerWidth;
    this.height = window.innerHeight;
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setSize(this.width, this.height);
    this.camera.aspect = this.width / this.height;
    this.camera.updateProjectionMatrix();
  }


  render() {
    var objectsToRemove = [];
    var curTime = Date.now();
    
    for(var i=0;i<this.objects.length;i++)
    {
      if(curTime > this.objects[i].lastUpdateTime + 1000) objectsToRemove.push(this.objects[i]);
    }
    
    for(var i=0;i<objectsToRemove.length;i++)
    {
      this.removeObject(objectsToRemove[i]);
    }

    requestAnimationFrame(this.render.bind(this)); //continuous rendering
    this.renderer.render(this.scene, this.camera);
  }


  //Objects

  getObjectWithId(id)
  {
    for(var i=0;i<this.objects.length;i++)
    {
      if(this.objects[i].objectID == id)
      {
        return this.objects[i];
      }
    }
    return null;
  }

  addObject(o)
  {
    //console.log("add object with id",o.objectID);
    this.scene.add(o);
    this.objects.push(o);
  }

  removeObject(o)
  {
    //console.log("remove object with id",o.objectID,this.objects.indexOf(o));
    this.scene.remove(o);
    this.objects.splice(this.objects.indexOf(o), 1);
  }

  clearObjects()
  {
    while(this.objects.length > 0) this.removeObject(this.objects[0]);
  }

  //Websocket
  connectWebSocket() {
      
    this.socket = new WebSocket("ws://" + location.host);
    this.socket.binaryType = "arraybuffer";

    let _app = this;

    this.socket.onopen = function(e) { _app.onSocketOpen(e) };
    this.socket.onclose = function(e) { _app.onSocketClose(e) };
    this.socket.onopen = function(e) { _app.onSocketOpen(e) };
    this.socket.onmessage = function(e) { _app.onSocketMessage(e) };

  }
  
  onSocketOpen (event) 
  {
    console.log("Connected to: " + event.currentTarget.url);
    this.socketFirstConnect = true;
  };

  onSocketClose(event) 
  {
    this.clearObjects();
    var _app = this;
    setTimeout(function () { _app.connectWebSocket(); }, 1000);
  };

  onSocketError(event)
  {
    if(this.socketFirstConnect) console.log("WebSocket Error: " + error);
    this.socketFirstConnect = false;
    this.clearObjects();
  }
  
  onSocketMessage(event) 
  {
    var data = event.data;

    var type = new Uint8Array(data.slice(0,1))[0];
    var objectID =  new Int32Array(data.slice(1,5))[0];
    var o = this.getObjectWithId(objectID);
    if(o == null)
    {
      //create object
      let c = new PObject(objectID, type, data);
      this.addObject(c);
    }else if(type == -1) //dataType CLEAR
    {
      clearObjects();
    }else
    {
      if(type == 0) //cloud
      {
        o.update(data);
      }
      else if(type == 1) //cluster
      {
        var state = new Int32Array(data.slice(5,9))[0];
        if(state == 2) //Will leave
        {
          this.removeObject(o);
        }
        else
        {
          o.update(data);
        }
      }
    }
  }

    /*
    var numExtraData = 0;

    var isCluster = dataType == 1;

    let wireframe;
    let geometry;
    let state;
    if(isCluster) // 0 = simple cloud, only id+cloud, 1 = cluster
    {

      console.log(id);

      numExtraData = 4+6*12;
      state = new Int32Array(data.slice(5,9))[0];
      var bounds = new Float32Array(data.slice(9,9+6*12));

      //Bounding box
      
      var boxSize = new THREE.Vector3().add(boxMax).sub(boxMin);
      var boxCenter = new THREE.Vector3().add(boxMin).add(boxMax).divideScalar(2);
      
      if(boxMin.x != 0) geometry.translate(boxCenter.x, boxCenter.y, boxCenter.z);
    }

    
    if(app.geometries[id] == undefined)
    {
      app.geometries[id] = new THREE.BufferGeometry();
      app.materials[id] = new THREE.PointsMaterial({
        size: 0.005,
        color: 0xffffff,
        transparent: true,
        opacity: 0.8
      });

      app.meshes[id] = new THREE.Points(app.geometries[id], app.materials[id]);
      app.scene.add(app.meshes[id]);

     if(isCluster) // 0 = simple cloud, only id+cloud, 1 = cluster
      {
         app.boundMeshes[id] = new THREE.Mesh( geometry, wireframe );
        app.scene.add( app.boundMeshes[id] );
      }
    }

    app.materials[id].color.setHSL(id/5.0,1,.5);
    app.meshes[id].material = app.materials[id];
    
    //console.log("update mesh : "+id+", data size "+vertices.length);

    app.geometries[id].setAttribute(
      "position",
      new THREE.BufferAttribute(vertices, 3)
    );


   if(isCluster) // 0 = simple cloud, only id+cloud, 1 = cluster
   {
      app.boundMeshes[id].geometry = geometry;
      app.boundMeshes[id].material.color.setHSL(id/5.0,state == 3?0:1,state == 0?1:state == 2?0:.5);
    }
  }
  */
}


var app = new App();