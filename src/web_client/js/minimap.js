/**
 * @file <client/js/minimap.js>
 * 
 * @author Genki Miyauchi - <g.miyauchi@sheffield.ac.uk>
 * 
 * MIT License
 * Copyright (c) 2021 NRL Lab
 */

var canvas;

var entities = new Map();

var OFFSET = 0;
var SCALE = 0;

/* ----------------------- */

var InitializeMinimap = function (minimap_panel) {
  var _width = minimap_panel.width() * 0.98;
  var _height = minimap_panel.height() * 0.98;

  OFFSET = _height / 2
  SCALE = _height / 6

  canvas = new fabric.StaticCanvas('mini-map',
    {
      // backgroundColor: 'rgb(225,225,225)',
      width: _width,
      height: _height,
      centeredRotation: true,
      centeredScaling: true
      // containerClassName: 'theCanvas',
      // preserveObjectStacking: true,
    });
}

function update_minimap() {

  // Loop all entities
    // If entity is box, get id
      // if id not in map, create new object
      // else, update object

  for (const key in sceneEntities) {
    if (sceneEntities.hasOwnProperty(key)) {
      const entity = sceneEntities[key].entity;
      if(entity.type == 'box') {

        if(key in entities) {
          // console.log("already here");
        } else {

          var box = new fabric.Rect({
            left: OFFSET + SCALE * entity.position.x,
            top: OFFSET - SCALE * entity.position.y,
            fill: '#333',
            width: SCALE * entity.scale.x,
            height: SCALE * entity.scale.y,
            originX: "center",
            originY: "center"
          });

          box.rotate(2 * Math.acos(entity.orientation.w) * 180 / Math.PI)
          
          entities[key] = box;
          canvas.add(box);
        }
      }
    }
  }

  canvas.renderAll()
}