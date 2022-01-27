/**
 * @file <client/js/three_scene.js>
 * 
 * @author Prajankya Sonar - <prajankya@gmail.com>
 * 
 * @project ARGoS3-Webviz <https://github.com/NESTlab/argos3-webviz>
 * 
 * MIT License
 * Copyright (c) 2020 NEST Lab
 */


var camera, cameraOrtho, cameraRobot, controls, renderer, stats, menuRenderer;
var scale;
var uuid2idMap = {};
var selectedEntities = {}

var mouseDownEvent = null

/* Main 3D scene */
var scene = new THREE.Scene();

/* Second scene to display HUD sprites */
var sceneOrtho = new THREE.Scene();

window.isInitialized = false;
window.isLoadingModels = false;

/* Initialize Three.js scene */
THREE.Object3D.DefaultUp.set(0, 0, 1);
scene.background = new THREE.Color(0x007f7f);

let iconGray = '/images/user_icon_gray.png';
let iconBlue = '/images/user_icon_blue.png';
let iconOrange = '/images/user_icon_orange.png';

// compute mouse position in normalized device coordinates
// (-1 to +1) for both directions.
// Used to raycasting against the interactive elements

let objsToTest = [];

const raycaster = new THREE.Raycaster();

const mouse = new THREE.Vector2();
mouse.x = mouse.y = null;

let selectState = false;

window.addEventListener( 'pointermove', ( event )=>{
	mouse.x = ( event.clientX / window.innerWidth ) * 2 - 1;
	mouse.y = - ( event.clientY / window.innerHeight ) * 2 + 1;
});

window.addEventListener( 'pointerdown', ()=> { selectState = true });

window.addEventListener( 'pointerup', ()=> { selectState = false });

window.addEventListener( 'touchstart', ( event )=> {
	selectState = true;
	mouse.x = ( event.touches[0].clientX / window.innerWidth ) * 2 - 1;
	mouse.y = - ( event.touches[0].clientY / window.innerHeight ) * 2 + 1;
});

window.addEventListener( 'touchend', ()=> {
	selectState = false;
	mouse.x = null;
	mouse.y = null;
});

/* ----------------------- */
var sceneEntities = [];

var InitializeThreejs = function (threejs_panel) {
  var _width = threejs_panel.width();
  var _height = threejs_panel.height();

  /* WebGLRenderer */
  renderer = new THREE.WebGLRenderer({
    antialias: true
  });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.outputEncoding = THREE.sRGBEncoding;

  /* Add canvas to page */
  renderer.setSize(_width, _height);
  threejs_panel.append(renderer.domElement);

  renderer.autoClear = false; // To allow render overlay

  /* Right click menu renderer */
  menuRenderer = new THREE.CSS2DRenderer();
  menuRenderer.domElement.id = 'dom_menu'
  menuRenderer.domElement.style.backgroundColor = 'rgba(0,0,0,0.4)';
  menuRenderer.domElement.style.display = 'none';

  /* Add canvas, and menu renderers to page */
  menuRenderer.setSize(_width, _height);
  threejs_panel.append(menuRenderer.domElement);

  /* Lights */
  var light = new THREE.DirectionalLight(0xffffff, 1);
  light.position.set(1, -1, 1);
  light.layers.enable(0);// enabled by default
  light.layers.enable(1);// All selectable objects
  scene.add(light);

  var light = new THREE.DirectionalLight(0x222222);
  light.position.set(-1, 1, -1);
  light.layers.enable(0);// enabled by default
  light.layers.enable(1);// All selectable objects
  scene.add(light);

  var light = new THREE.AmbientLight(0x333333);
  light.position.set(0, 0, 1);
  light.layers.enable(0);// enabled by default
  light.layers.enable(1);// All selectable objects
  scene.add(light);

  /* Add event hander for mouse left click */
  threejs_panel.click(onThreejsPanelMouseClick);

  /* Add event hander for mouse right click */
  threejs_panel.mouseup(onThreejsPanelMouseContextMenu);
  threejs_panel.mousedown(function (event) {
    mouseDownEvent = event
  });
}

function initSceneWithScale(_scale) {
  scale = _scale;

  /* Top-view camera */
  // camera = new THREE.PerspectiveCamera(45, window.threejs_panel.width() / window.threejs_panel.height(), 0.01, scale * 2500);
  camera = new THREE.PerspectiveCamera( 60, window.threejs_panel.width() / window.threejs_panel.height(), 0.1, 1000 );

  camera.position.set(-scale * 3, 0, scale * 5);

  camera.layers.enable(0); // enabled by default
  camera.layers.enable(1); // All selectable objects

  /* Orthogonal camera */
  cameraOrtho = new THREE.OrthographicCamera( - window.threejs_panel.width() / 2, window.threejs_panel.width() / 2, window.threejs_panel.height() / 2, - window.threejs_panel.height() / 2, 1, 100 );
  cameraOrtho.position.z = 10;
  
  cameraOrtho.layers.enable(0); // enabled by default
  cameraOrtho.layers.enable(1); // All selectable objects

  /* Robot perspective camera */
  cameraRobot = new THREE.PerspectiveCamera(60, window.threejs_panel.width() / window.threejs_panel.height(), 0.01, scale * 2500);

  cameraRobot.position.set(-scale * 3, 0, scale * 5);

  cameraRobot.layers.enable(0); // enabled by default
  cameraRobot.layers.enable(1); // All selectable objects

  // Controls
  // Possible types: OrbitControls, MapControls
  controls = new THREE.OrbitControls(camera, renderer.domElement);

  controls.enableDamping = true; // an animation loop is required when either damping or auto-rotation are enabled

  controls.dampingFactor = 0.05;
  controls.maxPolarAngle = Math.PI / 2;
  controls.screenSpacePanning = false;

  controls.minDistance = scale / 3;
  controls.maxDistance = scale * 2 * Math.max(window.experiment.data.arena.size.y,
    window.experiment.data.arena.size.x);

  var floor_found = false;
  window.experiment.data.entities.map(function (entity) {
    if (entity.type == "floor") {
      floor_found = true
    }
  })

  /* If no floor entity in experiment */
  if (!floor_found) {
    /* Ground plane */
    var plane_geometry = new THREE.BoxBufferGeometry(
      window.experiment.data.arena.size.x * scale,
      window.experiment.data.arena.size.y * scale,
      0.01
    );

    /* Bring to on top of zero*/
    plane_geometry.translate(0, 0, -0.005 * scale);

    new THREE.TextureLoader().load("/images/ground.png", function (texture) {
      texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
      texture.repeat.set(scale / 2.5, scale / 2.5);

      var material = new THREE.MeshPhongMaterial({
        map: texture
      })
      var plane = new THREE.Mesh(plane_geometry, material);
      plane.layers.set(0);

      scene.add(plane)
    });
  }

  /***************************/



  /* Main Block */

  const mainContainer = new ThreeMeshUI.Block({
    // ref: 'container',
    padding: 0.025,
    fontFamily: '/fonts/Roboto-msdf.json',
    fontTexture: '/fonts/Roboto-msdf.png',
    fontColor: new THREE.Color(0xffffff),
    fontSupersampling: true,
    backgroundOpacity: 0,
    alignContent: 'left',
  });
  mainContainer.position.set( -window.threejs_panel.width() / 2 + 180, window.threejs_panel.height() / 2 - 110, 0 );
  sceneOrtho.add(mainContainer);

  /* User Block */

  const userContainer = new ThreeMeshUI.Block({
    margin: 5,
    padding: 5,
    contentDirection: "row",
    backgroundOpacity: 0.9,
    borderRadius: [0, 0, 50, 0],
		borderWidth: 4,
		borderOpacity: 1,
  });
  mainContainer.add(userContainer);

    /* User Icon Block */

  const userIconContainer = new ThreeMeshUI.Block({
    margin: 2,
    padding: 0,
    borderRadius: 0,
    borderWidth: 3,
    backgroundOpacity: 1,
    backgroundColor: new THREE.Color( 0.9, 0.9, 0.9 ),
  });
  userContainer.add(userIconContainer);

  window.userIcon = new ThreeMeshUI.Block({
    margin: 0,
    padding: 10,
    height: 80,
    width: 75,
  });
  userIconContainer.add(window.userIcon);

  new THREE.TextureLoader().load(iconGray, (texture) => {
    // necessary for backgroundSize: 'contain'
		texture.wrapS = THREE.RepeatWrapping;
		texture.wrapT = THREE.RepeatWrapping;

    window.userIcon.set({
      backgroundTexture: texture,
      backgroundSize: 'stretch',
    });
  });

    /* User Info Block */

  const userInfoContainer = new ThreeMeshUI.Block({
    margin: 2,
    contentDirection: "column",
    alignContent: 'left',
    backgroundOpacity: 0,
    borderOpacity: 0,
    borderWidth: 1,
    borderRadius: 0,
  });
  userContainer.add(userInfoContainer);

    /* User Follower Block */

  const userTopContainer = new ThreeMeshUI.Block({
    contentDirection: "row",
    backgroundOpacity: 0,
    fontSize: 24,
  });
  userInfoContainer.add(userTopContainer);

      /* User Name Block */

  const userNameContainer = new ThreeMeshUI.Block({
    width: 80,
    height: 40,
    justifyContent: 'center',
    alignContent: 'center',
    backgroundOpacity: 0,
    borderRadius: 0,
  });
  userTopContainer.add(userNameContainer);

  window.userName = new ThreeMeshUI.Text({
    content: "User -",
  });

  userNameContainer.add(window.userName);

      /* User Follower Label Block */

  const userFollowerContainer = new ThreeMeshUI.Block({
    contentDirection: "column",
    backgroundOpacity: 0,
  });
  userTopContainer.add(userFollowerContainer);

  const userFollowerLabelContainer = new ThreeMeshUI.Block({
    width: 160,
    height: 20,
    justifyContent: 'center',
    alignContent: 'center',
    backgroundOpacity: 0,
    borderRadius: 0,
  });
  userFollowerContainer.add(userFollowerLabelContainer);

  userFollowerLabelContainer.add(
		new ThreeMeshUI.Text({
      content: "Follower / Require ",
      fontSize: 16,
		}),
  );

      /* User Follower Count Block */

  const userFollowerCountContainer = new ThreeMeshUI.Block({
    width: 160,
    height: 40,
    contentDirection: "row",
    justifyContent: 'center',
    alignContent: 'center',
    fontSize: 28,
    backgroundOpacity: 0,
  });
  userFollowerContainer.add(userFollowerCountContainer);

  window.numFollowers = new ThreeMeshUI.Text({
    content: "-",
  });

  window.numTaskRequire = new ThreeMeshUI.Text({
    content: "-",
  });

  const userFollowerCount = new ThreeMeshUI.Block({
    width: 70,
    height: 40,
    justifyContent: 'center',
    alignContent: 'center',
    fontSize: 36,
    backgroundOpacity: 0,
  });
  userFollowerCountContainer.add(userFollowerCount);
  userFollowerCount.add(window.numFollowers);

  const userFollowerSlashLabel = new ThreeMeshUI.Block({
    width: 15,
    height: 40,
    // margin: 10,
    justifyContent: 'center',
    alignContent: 'center',
    fontSize: 32,
    backgroundOpacity: 0,
  });
  userFollowerCountContainer.add(userFollowerSlashLabel);
  userFollowerSlashLabel.add(
    new ThreeMeshUI.Text({
      content: " / ",
		}),
  );

  const userRequiredCount = new ThreeMeshUI.Block({
    width: 70,
    height: 40,
    // margin: 10,
    justifyContent: 'center',
    alignContent: 'center',
    fontSize: 36,
    backgroundOpacity: 0,
  });
  userFollowerCountContainer.add(userRequiredCount);
  userRequiredCount.add(window.numTaskRequire);

    /* User Task Block */

  const userTaskContainer = new ThreeMeshUI.Block({
    // margin: 10,
    contentDirection: "row",
    // alignContent: 'left',
    backgroundOpacity: 0,
  });
  userInfoContainer.add(userTaskContainer);

      /* User Progress Bar Block */

  /* Progress Bar background */
  window.progressBarWidth = 150;
      
  const userProgressBarContainer = new ThreeMeshUI.Block({
    width: window.progressBarWidth,
    height: 20,
    // margin: 10,
    justifyContent: 'center',
    alignContent: 'center',
    backgroundOpacity: 0,
  });
  userTaskContainer.add(userProgressBarContainer);

  const progressBar = new ThreeMeshUI.Block({
		width: window.progressBarWidth,
		height: 10,
    margin: 0,
		padding: 0,
		justifyContent: 'center',
		alignContent: 'left',
    backgroundColor: new THREE.Color( 0.4, 0.4, 0.4 ),
    backgroundOpacity: 1,
    borderWidth: 0,
    borderRadius: 5,
	});
  userProgressBarContainer.add(progressBar);

  /* Progress bar foreground */
  window.progress = new ThreeMeshUI.Block({
    width: 0.001,
    height: 10,
    margin: 0,
    padding: 0,
    backgroundColor: new THREE.Color( 0, 1, 0 ),
    backgroundOpacity: 0,
  });
  progressBar.add(window.progress);

      /* User Progress Percent Block */

  const userProgressPercentContainer = new ThreeMeshUI.Block({
    width: 60,
    height: 30,
    // margin: 10,
    justifyContent: 'center',
    alignContent: 'right',
    backgroundOpacity: 0,
    fontSize: 20,
  });
  userTaskContainer.add(userProgressPercentContainer);

  window.numProgress = new ThreeMeshUI.Text({
    // fontColor: new THREE.Color( 0, 0, 0 ),
    content: "-",
  });

  userProgressPercentContainer.add(

    window.numProgress,

		new ThreeMeshUI.Text({
      content: " %",
		}),
  );

  /* Other User Block */

  const otherUserContainer = new ThreeMeshUI.Block({
    margin: 5,
    padding: 5,
    contentDirection: "row",
    backgroundOpacity: 0.9,
    borderRadius: [0, 0, 40, 0],
		borderWidth: 4,
		borderOpacity: 1,
  });
  mainContainer.add(otherUserContainer);

    /* Other User Icon Block */

  const otherUserIconContainer = new ThreeMeshUI.Block({
    margin: 2,
    padding: 0,
    borderRadius: 0,
    borderWidth: 3,
    backgroundOpacity: 1,
    backgroundColor: new THREE.Color( 0.9, 0.9, 0.9 ),
  });
  otherUserContainer.add(otherUserIconContainer);

  let iconScale = 0.7;

  window.otherUserIcon = new ThreeMeshUI.Block({
    margin: 0,
    padding: 10,
    height: 80 * iconScale,
    width: 75 * iconScale,
  });
  otherUserIconContainer.add(window.otherUserIcon);

  new THREE.TextureLoader().load(iconGray, (texture) => {
    // necessary for backgroundSize: 'contain'
		texture.wrapS = THREE.RepeatWrapping;
		texture.wrapT = THREE.RepeatWrapping;

    window.otherUserIcon.set({
      backgroundTexture: texture,
      backgroundSize: 'stretch',
    });
  });

    /* Other User Info Block */

  const otherUserInfoContainer = new ThreeMeshUI.Block({
    margin: 2,
    contentDirection: "column",
    alignContent: 'left',
    backgroundOpacity: 0,
    borderOpacity: 0,
    borderWidth: 1,
    borderRadius: 0,
  });
  otherUserContainer.add(otherUserInfoContainer);

    /* Other User Follower Block */

  const otherUserTopContainer = new ThreeMeshUI.Block({
    contentDirection: "row",
    backgroundOpacity: 0,
    fontSize: 24,
  });
  otherUserInfoContainer.add(otherUserTopContainer);

      /* Other User Name Block */

  const otherUserNameContainer = new ThreeMeshUI.Block({
    width: 80,
    height: 40,
    justifyContent: 'center',
    alignContent: 'center',
    backgroundOpacity: 0,
    borderRadius: 0,
  });
  otherUserTopContainer.add(otherUserNameContainer);

  window.otherUserName = new ThreeMeshUI.Text({
    content: "User -",
  }),

  otherUserNameContainer.add(window.otherUserName);

      /* Other User Follower Label Block */

  const otherUserFollowerContainer = new ThreeMeshUI.Block({
    contentDirection: "column",
    backgroundOpacity: 0,
  });
  otherUserTopContainer.add(otherUserFollowerContainer);

  const otherUserFollowerLabelContainer = new ThreeMeshUI.Block({
    width: 160,
    height: 20,
    justifyContent: 'center',
    alignContent: 'center',
    backgroundOpacity: 0,
    borderRadius: 0,
  });
  otherUserFollowerContainer.add(otherUserFollowerLabelContainer);

  otherUserFollowerLabelContainer.add(
		new ThreeMeshUI.Text({
      content: "Follower / Require ",
      fontSize: 16,
		}),
  );

      /* Other User Follower Count Block */

  const otherUserFollowerCountContainer = new ThreeMeshUI.Block({
    width: 160,
    height: 40,
    contentDirection: "row",
    justifyContent: 'center',
    alignContent: 'center',
    fontSize: 28,
    backgroundOpacity: 0,
  });
  otherUserFollowerContainer.add(otherUserFollowerCountContainer);

  window.numOtherFollowers = new ThreeMeshUI.Text({
    content: "-",
  });

  window.numOtherTaskRequire = new ThreeMeshUI.Text({
    content: "-",
  });

  const otherUserFollowerCount = new ThreeMeshUI.Block({
    width: 70,
    height: 40,
    justifyContent: 'center',
    alignContent: 'center',
    fontSize: 28,
    backgroundOpacity: 0,
  });
  otherUserFollowerCountContainer.add(otherUserFollowerCount);
  otherUserFollowerCount.add(window.numOtherFollowers);

  const otherUserFollowerSlashLabel = new ThreeMeshUI.Block({
    width: 15,
    height: 40,
    justifyContent: 'center',
    alignContent: 'center',
    fontSize: 28,
    backgroundOpacity: 0,
  });
  otherUserFollowerCountContainer.add(otherUserFollowerSlashLabel);
  otherUserFollowerSlashLabel.add(
    new ThreeMeshUI.Text({
      content: " / ",
		}),
  );

  const otherUserRequiredCount = new ThreeMeshUI.Block({
    width: 70,
    height: 40,
    justifyContent: 'center',
    alignContent: 'center',
    fontSize: 28,
    backgroundOpacity: 0,
  });
  otherUserFollowerCountContainer.add(otherUserRequiredCount);
  otherUserRequiredCount.add(window.numOtherTaskRequire);

  
  /* Send Robot Block */

  const sendContainer = new ThreeMeshUI.Block({
    // ref: 'container',
    padding: 0.025,
    fontFamily: '/fonts/Roboto-msdf.json',
    fontTexture: '/fonts/Roboto-msdf.png',
    fontColor: new THREE.Color(0xffffff),
    backgroundOpacity: 0.1,
    alignContent: 'left',
  });
  sendContainer.position.set( -window.threejs_panel.width() / 2 + 580, window.threejs_panel.height() / 2 - 100, 0 );
  sceneOrtho.add(sendContainer);

    /* Label Block */

  const sendLabelContainer = new ThreeMeshUI.Block({
    width: 80,
    height: 20,
    margin: 10,
    justifyContent: 'center',
    alignContent: 'center',
    backgroundOpacity: 0.1,
    // borderRadius: [0, 50, 0, 50],
		borderWidth: 1,
		// borderColor: new THREE.Color( 0, 0.5, 1 ),
		borderOpacity: 1,
  });
  sendContainer.add(sendLabelContainer);

  sendLabelContainer.add(
    new ThreeMeshUI.Text({
      content: "Followers:",
      fontSize: 16,
			fontColor: new THREE.Color( 0, 0, 0 ),
    }),
  );

    /* Control Block */

  const sendControlContainer = new ThreeMeshUI.Block({
    margin: 10,
    contentDirection: "row",
    backgroundOpacity: 0.1,
    // borderRadius: [0, 50, 0, 50],
		borderWidth: 1,
		// borderColor: new THREE.Color( 0, 0.5, 1 ),
		borderOpacity: 1,
  });
  sendContainer.add(sendControlContainer);

      /* Count Block */

    const sendCountContainer = new ThreeMeshUI.Block({
      width: 80,
      height: 40,
      margin: 10,
      justifyContent: 'center',
      alignContent: 'center',
      backgroundOpacity: 0.1,
    });
    sendControlContainer.add(sendCountContainer);

      /* Counter */
    window.toSendCount = 0;

    window.sendCountLabel = new ThreeMeshUI.Text({
      content: window.toSendCount.toString(),
      fontSize: 28,
      fontColor: new THREE.Color( 0, 0, 0 ),
    });

    sendCountContainer.add(window.sendCountLabel);

      /* Toggle Block */

    const sendToggleContainer = new ThreeMeshUI.Block({
      margin: 10,
      alignContent: 'right',
      backgroundOpacity: 0.1,
      // borderRadius: [0, 50, 0, 50],
      borderWidth: 1,
      // borderColor: new THREE.Color( 0, 0.5, 1 ),
      borderOpacity: 1,
    });
    sendControlContainer.add(sendToggleContainer);

    const hoveredStateAttributes = {
      state: "hovered",
      attributes: {
        offset: 0.035,
        backgroundColor: new THREE.Color( 0x999999 ),
        backgroundOpacity: 1,
        fontColor: new THREE.Color( 0xffffff )
      },
    };
  
    const idleStateAttributes = {
      state: "idle",
      attributes: {
        offset: 0.035,
        backgroundColor: new THREE.Color( 0x666666 ),
        backgroundOpacity: 0.3,
        fontColor: new THREE.Color( 0xffffff )
      },
    };

        /* Add Block */

    const sendAddButton = new ThreeMeshUI.Block({
      width: 30,
      height: 30,
      margin: 5,
      justifyContent: 'center',
      alignContent: 'center',
      fontSize: 28,
      borderRadius: 15,
      // backgroundOpacity: 1,
    })

    sendAddButton.add(
      new ThreeMeshUI.Text({
        content: "+",
        // fontColor: new THREE.Color( 1, 1, 1 ),
      }),
    );

        /* Subtract Block */

    const sendSubtractButton = new ThreeMeshUI.Block({
      width: 30,
      height: 30,
      margin: 5,
      justifyContent: 'center',
      alignContent: 'center',
      fontSize: 28,
      borderRadius: 15,
      // backgroundOpacity: 1,
    })

    sendSubtractButton.add(
      new ThreeMeshUI.Text({
        content: "-",
        // fontColor: new THREE.Color( 1, 1, 1 ),
      }),
    );

    const selectedAttributes = {
      offset: 0.02,
      backgroundColor: new THREE.Color( 0x777777 ),
      fontColor: new THREE.Color( 0x222222 )
    };
  
    sendAddButton.setupState({
      state: "selected",
      attributes: selectedAttributes,
      onSet: ()=> {
        console.log("Add Selected");
        window.toSendCount++;
        window.sendCommand['number'] = window.toSendCount;
        window.sendCountLabel.set({
          content: window.toSendCount.toString(),
        });
      }
    });
    sendAddButton.setupState( hoveredStateAttributes );
    sendAddButton.setupState( idleStateAttributes );

    sendSubtractButton.setupState({
      state: "selected",
      attributes: selectedAttributes,
      onSet: ()=> {
        console.log("Subtract Selected");
        if(window.toSendCount > 0) {
          window.toSendCount--;
          window.sendCommand['number'] = window.toSendCount;
          window.sendCountLabel.set({
            content: window.toSendCount.toString(),
          });
        }
      }
    });
    sendSubtractButton.setupState( hoveredStateAttributes );
    sendSubtractButton.setupState( idleStateAttributes );

    sendToggleContainer.add(sendAddButton, sendSubtractButton);
    objsToTest.push(sendAddButton, sendSubtractButton);

      /* Confirm Block */

    const sendConfirmButton = new ThreeMeshUI.Block({
      width: 80,
      height: 40,
      margin: 5,
      justifyContent: 'center',
      alignContent: 'center',
      fontSize: 20,
      borderRadius: 15,
      // backgroundOpacity: 1,
    });

    sendConfirmButton.add(
      new ThreeMeshUI.Text({
        content: "Send",
        // fontColor: new THREE.Color( 1, 1, 1 ),
      }),
    );

    sendConfirmButton.setupState({
      state: "selected",
      attributes: selectedAttributes,
      onSet: ()=> {
        console.log("Confirm Selected");
        if(window.toSendCount > 0) {
          window.sendFlag = true;
        }
      }
    });
    sendConfirmButton.setupState( hoveredStateAttributes );
    sendConfirmButton.setupState( idleStateAttributes );

    sendControlContainer.add(sendConfirmButton);
    objsToTest.push(sendConfirmButton);

  /***************************/

}

function cleanUpdateScene() {
  window.isLoadingModels = true;
  /* Remove all meshes */

  for (const key in sceneEntities) {
    if (sceneEntities.hasOwnProperty(key)) {

      const object = scene.getObjectByProperty('uuid', sceneEntities[key].uuid);

      if (object) {
        object.geometry.dispose();
        object.material.dispose();
        scene.remove(object);
      }
    }
  }
  /* reset Map */
  uuid2idMap = {};
  selectedEntities = {}

  var count = window.experiment.data.entities.length
  window.experiment.data.entities.map((entity) => {

    if (entity) { //Neglect Null Entities
      GetEntity(entity, scale, function (entityObject) {
        if (entityObject) {
          var mesh = entityObject.getMesh()

          if (mesh) {
            entityObject.uuid = mesh.uuid

            /* Copy basic properties from Argos entities to threejs objects */
            entityObject.id = entity.id;
            entityObject.type_description = entity.type;

            /* UUID to ID map */
            uuid2idMap[mesh.uuid] = entity.id;

            sceneEntities[entity.id] = entityObject;

            /* Its not an object with "is_movable", so considering it movable(robots) */
            if (typeof entity.is_movable === 'undefined' || entity.is_movable === null) {
              /* Hardcoded floor entity in non-selectable entity */
              if (entity.type == "floor") {
                /* Non selectable */
                mesh.layers.set(0);
              } else {
                /* Add to selectable layer */
                mesh.layers.set(1);
                mesh.traverse(function (child) { child.layers.set(1) })
              }
            } else {
              /* If "is_movable" is true */
              if (entity.is_movable && entity.is_movable === true) {
                /* Add to selectable layer */
                mesh.layers.set(1);
              } else {
                /* Non selectable */
                mesh.layers.set(0);
              }
            }

            scene.add(mesh);
          }
        }

        count--;

        if (count == 0) { // Finished loading all models
          window.isLoadingModels = false;
        }
      });
    } else {
      console.error("Entity is null");
    }
  })
}

function onThreejsPanelResize() {
  var _width = window.threejs_panel.width();
  var _height = window.threejs_panel.height();

  camera.aspect = _width / _height
  camera.updateProjectionMatrix();

  renderer.setSize(_width, _height);
  menuRenderer.setSize(_width, _height);
}

function onThreejsPanelMouseContextMenu(event) {
  event.preventDefault();
  if (!event.originalEvent || event.which != 3) {
    return
  }
  if (mouseDownEvent.offsetX !== event.offsetX ||
    mouseDownEvent.offsetY !== event.offsetY) {
    /* User is panning */
    return
  }

  var contextMenuConfig = {
    selector: '#dom_menu',
    appendTo: '#dom_menu',
    trigger: 'none',
    events: {
      show: function () {
        $("#dom_menu").show()
      },
      hide: function (options) {
        $("#dom_menu").hide()
        $.contextMenu('destroy'); // Delete menu
      }
    },
    position: function (opt, x, y) {
      var origX = x - window.threejs_panel.offset().left - 4
      var origY = y - window.threejs_panel.offset().top - 4

      if (origX + opt.$menu.width() > window.threejs_panel.width()) {
        origX -= opt.$menu.width()
      }
      if (origY + opt.$menu.height() > window.threejs_panel.height()) {
        origY -= opt.$menu.height() + 10
      }

      opt.$menu.css({
        top: origY,
        left: origX
      });
    }
  };

  var mouse = new THREE.Vector2();
  var raycaster = new THREE.Raycaster();
  raycaster.layers.set(1);// Allow to select only in layer 1

  // Convert to -1 to +1
  mouse.x = (event.offsetX / window.threejs_panel.width()) * 2 - 1;
  mouse.y = - (event.offsetY / window.threejs_panel.height()) * 2 + 1;


  // update the picking ray with the camera and mouse position
  raycaster.setFromCamera(mouse, camera);

  // calculate objects intersecting the picking ray 
  // true for recursive(nested)
  var intersects = raycaster.intersectObjects(scene.children, true);

  if (intersects.length > 0) {
    /* Get only the top object */
    var object = intersects[0].object
    /* Get root object, whole parent is Scene */
    while (object.parent.type != "Scene") {
      object = object.parent
    }

    /* Already selected */
    if (selectedEntities[object.uuid]) {
      contextMenuConfig.items = {
        "deselect": {
          name: "Deselect",
          accesskey: 'd',
          callback: function () {
            deselectObject(object)
          }
        },
      };
    } else {
      /* Not selected already */
      contextMenuConfig.items = {
        "select": {
          name: "Select",
          accesskey: 's',
          callback: function () {
            /* deselect all currently selected entities */
            for (const uuid in selectedEntities) {
              if (selectedEntities.hasOwnProperty(uuid)) {
                deselectObjectByUUID(uuid)
              }
            }
            selectObject(object)
          }
        },
      };

    }
  } else { // No items intersects
    /* Only one entity is selected */
    if (Object.keys(selectedEntities).length == 1) {
      var ids = [];

      for (const uuid in selectedEntities) {
        if (selectedEntities.hasOwnProperty(uuid)) {
          if (uuid2idMap[uuid]) { // Found object
            ids.push(uuid2idMap[uuid]);
          }// selected object is no more in scene, dont know what to do...
        }
      }

      contextMenuConfig.items = {
        "move": {
          name: "Move selected here",
          accesskey: 'm',
          callback: function () {
            var pos = get2DProjectedPosition(mouse, sceneEntities[ids[0]]);
            var mesh = sceneEntities[ids[0]].getMesh()

            window.wsp.sendPacked({
              command: 'moveEntity',
              entity_id: ids[0],
              position: {
                x: pos.x,
                y: pos.y,
                z: pos.z
              },
              orientation: {
                x: mesh.quaternion._x,
                y: mesh.quaternion._y,
                z: mesh.quaternion._z,
                w: mesh.quaternion._w
              }
            });
          }
        },
      };
    }
  }

  if (contextMenuConfig.items) {
    $.contextMenu(contextMenuConfig);

    /* Show menu */
    $('#dom_menu').contextMenu({ x: event.clientX, y: event.clientY });
  }
}

function onThreejsPanelMouseClick(event) {
  var mouse = new THREE.Vector2();
  var raycaster = new THREE.Raycaster();
  raycaster.layers.set(1);// Allow to select only in layer 1


  // Convert to -1 to +1
  mouse.x = (event.offsetX / window.threejs_panel.width()) * 2 - 1;
  mouse.y = - (event.offsetY / window.threejs_panel.height()) * 2 + 1;


  // update the picking ray with the camera and mouse position
  raycaster.setFromCamera(mouse, camera);

  // calculate objects intersecting the picking ray // true for recursive(nested)
  var intersects = raycaster.intersectObjects(scene.children, true);

  if (intersects.length > 0) {
    /* Get only the top object */
    var object = intersects[0].object
    /* Get root object, whole parent is Scene */
    while (object.parent.type != "Scene") {
      object = object.parent
    }

    /* Already selected */
    if (selectedEntities[object.uuid]) {
      if (event.shiftKey) {
        deselectObject(object)
      }
    } else {/* Object not already selected */
      if (event.shiftKey) {
        /* deselect all currently selected entities */
        for (const uuid in selectedEntities) {
          if (selectedEntities.hasOwnProperty(uuid)) {
            deselectObjectByUUID(uuid)
          }
        }

        /* Add to selection */
        selectObject(object)
      }
    }
  } else { // No object intersected
    var ids = [];

    for (const uuid in selectedEntities) {
      if (selectedEntities.hasOwnProperty(uuid)) {
        if (uuid2idMap[uuid]) { // Found object
          ids.push(uuid2idMap[uuid]);
        }// selected object is no more in scene, dont know what to do...
      }
    }


    /* Move object if  only control pressed, and one object selected */
    if (event.ctrlKey &&
      !event.altKey &&
      !event.shiftKey &&
      ids.length == 1) {

      var pos = get2DProjectedPosition(mouse, sceneEntities[ids[0]]);
      var mesh = sceneEntities[ids[0]].getMesh()

      window.wsp.sendPacked({
        command: 'moveEntity',
        entity_id: ids[0],
        position: {
          x: pos.x,
          y: pos.y,
          z: pos.z
        },
        orientation: {
          x: mesh.quaternion._x,
          y: mesh.quaternion._y,
          z: mesh.quaternion._z,
          w: mesh.quaternion._w
        }
      });
    }
  }
}

function selectObject(object) {
  if (!selectedEntities[object.uuid]) {
    var boundingBox = new THREE.BoxHelper(object, 0x000000);
    selectedEntities[object.uuid] = boundingBox

    scene.add(boundingBox);
  }
}

function deselectObjectByUUID(uuid) {
  if (selectedEntities[uuid]) {
    /* remove from selection */
    var boundingBox = selectedEntities[uuid]

    boundingBox.geometry.dispose();
    boundingBox.material.dispose();
    scene.remove(boundingBox);

    delete selectedEntities[uuid];
  }
}

function deselectObject(object) {
  deselectObjectByUUID(object.uuid)
}

function get2DProjectedPosition(mouse, object) {
  /* Object's Z plane */
  var z_plane = object.mesh.position.z / scale;

  var mouse_point = new THREE.Vector3();
  mouse_point.x = mouse.x;
  mouse_point.y = mouse.y;
  mouse_point.z = z_plane;

  var pos = new THREE.Vector3();


  /* Get point under the mouse */
  mouse_point.unproject(camera);
  mouse_point.sub(camera.position).normalize();
  var distance = (z_plane - camera.position.z) / mouse_point.z;
  pos.copy(camera.position).add(mouse_point.multiplyScalar(distance));

  /* divide by scale to convert back to units from server */
  pos.divideScalar(scale)

  return pos
}

function animate() {
  requestAnimationFrame(animate);
  ThreeMeshUI.update();
  updateButtons();
  updateCommands();
  controls.update();
  cameraUpdate();
  render();
  // update_minimap();
}

// Called in the loop, get intersection with either the mouse or the VR controllers,
// then update the buttons states according to result

function updateButtons() {

  /* Adjust mouse position to match UI */

  // console.log(mouse.x);
  // console.log(mouse.y);

  const newMouse = new THREE.Vector2();
  newMouse.copy(mouse);

  const translate = new THREE.Vector2(0.25,0.065);
  newMouse.add(translate);

  const resize = new THREE.Vector2(0.75,0.935);
  newMouse.divide(resize);

  // console.log(newMouse.x);
  // console.log(newMouse.y);

	// Find closest intersecting object

	let intersect;

	if ( newMouse.x !== null && newMouse.y !== null ) {

		raycaster.setFromCamera( newMouse, cameraOrtho );

		intersect = raycast();

	};

	// Update targeted button state (if any)

	if ( intersect && intersect.object.isUI ) {

    // Check if a leader is selected
    if(window.target != "") {

      // console.log(intersect.object);

      if ( selectState ) {

        // Component.setState internally call component.set with the options you defined in component.setupState
        intersect.object.setState( 'selected' );
  
      } else {
  
        // Component.setState internally call component.set with the options you defined in component.setupState
        intersect.object.setState( 'hovered' );
  
      };
    };
	};

	// Update non-targeted buttons state

	objsToTest.forEach( (obj)=> {

		if ( (!intersect || obj !== intersect.object) && obj.isUI ) {

			// Component.setState internally call component.set with the options you defined in component.setupState
			obj.setState( 'idle' );

		};

	});

};

function raycast() {

	return objsToTest.reduce( (closestIntersection, obj)=> {

		const intersection = raycaster.intersectObject( obj, true );

		if ( !intersection[0] ) return closestIntersection

		if ( !closestIntersection || intersection[0].distance < closestIntersection.distance ) {

			intersection[0].object = obj;

			return intersection[0]

		} else {

			return closestIntersection

		};

	}, null );

};

function updateCommands() {

  window.keyboard.update();

  /* Build paacket to send */
  var packet = {
                client: window.client_id,
                username: window.username,
                robot: window.target 
               };
  var commands = [];

  /* Check connect command (select_leader) */
  if( window.connectFlag ) {
    commands.push(window.connectCommand);
    window.connectFlag = false;
  }

  /* Check if task signal is toggelled (start/stop) */
  if( window.taskFlag ) {
    commands.push(window.taskCommand);
    window.taskFlag = false;
  }

  /* Check for a new request command */
  if( window.requestFlag ) {
    commands.push(window.requestCommand);
    window.requestFlag = false;
    console.log(packet);
  }

  /* Check for a new send command */
  if( window.sendFlag ) {
    commands.push(window.sendCommand);
    window.sendFlag = false;
    console.log(packet);
  }

  /* Check movement command */
  if( keyboard.pressed("W") ) {

    console.log("up pressed");
    let data = {
                command: 'move',
                direction: 'U'  // Up
               }
    commands.push(data);

  } else if( keyboard.pressed("S") ) {

    console.log("down pressed");
    let data = {
                command: 'move',
                direction: 'D'  // Down
               }
    commands.push(data);

  } else if( keyboard.pressed("A") ) {

    console.log("left pressed");
    let data = {
                command: 'move',
                direction: 'L'  // Left
                }
    commands.push(data);

  } else if( keyboard.pressed("D") ) {

    console.log("right pressed");
    let data = {
                command: 'move',
                direction: 'R'  // Right
               }
    commands.push(data);

  } else {

    if(window.target != '') {
      let data = {
                  command: 'move',
                  direction: 'S'  // Stop
                 }
      commands.push(data);
    }

  }

  packet['commands'] = commands;
  window.wsp.sendPacked(packet);
  // console.log(packet);
}


function cameraUpdate() {

  /* Find selected leader position and orientation */
  if(window.target != '') {
    if(sceneEntities.hasOwnProperty(window.target)) {

      // Update camera 
      const relativeCameraOffset = new THREE.Vector3(0,0,3);

      const object = scene.getObjectByProperty('uuid', sceneEntities[window.target].uuid);
      const cameraOffset = relativeCameraOffset.applyMatrix4( object.matrixWorld );
  
      cameraRobot.position.x = cameraOffset.x;
      cameraRobot.position.y = cameraOffset.y;
      cameraRobot.position.z = cameraOffset.z;
  
      const relativeFocusOffset = new THREE.Vector3(20,0,0);
      const focusOffset = relativeFocusOffset.applyMatrix4( object.matrixWorld );

      cameraRobot.lookAt( focusOffset );
    }
  } else {

  }
}


function render() {
  /* Experiment is initialized */
  if (window.experiment &&
    window.experiment.data &&
    window.experiment.data.entities &&
    window.isLoadingModels == false) {

    /* Entities count changed, clean and render again */
    if (window.experiment.data.entities.length != Object.keys(sceneEntities).length) {
      cleanUpdateScene();
      return; // Go to load models, do not update
    }

    /* Call update of each entity */
    window.experiment.data.entities.map((entity) => {
      sceneEntities[entity.id].update(entity, scale);
    });

    /* Update all bounding boxes */
    for (const uuid in selectedEntities) {
      if (selectedEntities.hasOwnProperty(uuid)) {
        selectedEntities[uuid].update();
      }
    }

    /* Update HUD values */
    if(window.target != '') {
      if(sceneEntities.hasOwnProperty(window.target)) {

        let num_followers = sceneEntities[window.target].entity.user_data.num_followers;
        let num_task_require = sceneEntities[window.target].entity.user_data.num_task_require;
        let num_task_demand = sceneEntities[window.target].entity.user_data.num_task_demand;
        let num_init_task_demand = sceneEntities[window.target].entity.user_data.num_init_task_demand;

        let num_other_followers = sceneEntities[window.target].entity.user_data.num_other_followers;
        let num_other_task_require = sceneEntities[window.target].entity.user_data.num_other_task_require;

        window.numFollowers.set({
          content: num_followers.toString(),
        });

        if(num_task_require == 0) {
          window.numTaskRequire.set({
            content: "-",
          });
        } else {
          window.numTaskRequire.set({
            content: num_task_require.toString(),
          });
        }

        if(num_init_task_demand == 0) {
          window.numProgress.set({
            content: "-",
          });

          window.progress.set({
            width: 0.001,
            backgroundOpacity: 0,
          });
        } else {
          let num_progress = 1 - num_task_demand / num_init_task_demand;

          window.numProgress.set({
            content: Math.floor(num_progress.toString() * 100).toString(),
          });

          if(num_progress != 0) {
            window.progress.set({
              width: num_progress * window.progressBarWidth,
              backgroundOpacity: 1,
            });
          }
        }

        if(num_other_followers == 0) {
          window.numOtherFollowers.set({
            content: "-",
          });
        } else {
          window.numOtherFollowers.set({
            content: num_other_followers.toString(),
          });
        }

        if(num_other_task_require == 0) {
          window.numOtherTaskRequire.set({
            content: "-",
          });
        } else {
          window.numOtherTaskRequire.set({
            content: num_other_task_require.toString(),
          });
        }
      }
    } else {

      /* Clear values */
      window.numFollowers.set({
        content: "-",
      });

      window.numTaskRequire.set({
        content: "-",
      });

      window.numOtherFollowers.set({
        content: "-",
      });

      window.numOtherTaskRequire.set({
        content: "-",
      });

      window.numProgress.set({
        content: "-",
      });

      window.progress.set({
        width: 0.001,
        backgroundOpacity: 0,
      });
    }

    /* Update icon */
    if(window.targetChanged) {

      var icon1 = iconGray;
      var icon2 = iconGray;
      var user1 = 'User -';
      var user2 = 'User -';

      if(window.target === "L1") {
        icon1 = iconBlue;
        icon2 = iconOrange;
        user1 = 'User 1';
        user2 = 'User 2';
      } else if(window.target === "L2") {
        icon1 = iconOrange;
        icon2 = iconBlue;
        user1 = 'User 2';
        user2 = 'User 1';
      }

      new THREE.TextureLoader().load(icon1, (texture) => {
        window.userIcon.set({
          backgroundTexture: texture,
        });
      });
      new THREE.TextureLoader().load(icon2, (texture) => {
        window.otherUserIcon.set({
          backgroundTexture: texture,
        });
      });

      window.userName.set({
        content: user1,
      });
      window.otherUserName.set({
        content: user2,
      });

      window.targetChanged = false;
    }

    // console.log(window.experiment.data);
  }

  if(window.connected) {
    renderer.render(scene, cameraRobot);
    menuRenderer.render(scene, cameraRobot);
  } else {
    renderer.render(scene, camera);
    menuRenderer.render(scene, camera);
  }

  renderer.clearDepth();
  renderer.render( sceneOrtho, cameraOrtho );

}
