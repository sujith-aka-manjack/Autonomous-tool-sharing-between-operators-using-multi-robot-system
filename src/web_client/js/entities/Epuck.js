/**
 * @file <client/js/entities/Epuck.js>
 * 
 * @author Genki Miyauchi - <g.miyauchi@sheffield.ac.uk>
 */

class Epuck {
  constructor(entity, scale, EntityLoadingFinishedFn) {
    this.scale = scale;
    this.entity = entity;
    this.lines = [];

    /* Scale to convert from mm to scale used here */
    var UNIT_SCALE = 0.001 * scale

    var that = this;
    var geometry = new THREE.CylinderBufferGeometry(
      35 * UNIT_SCALE,
      35 * UNIT_SCALE,
      86 * UNIT_SCALE,
      32
    );

    geometry.rotateX(-1.572);

    /* Bring to on top of zero*/
    geometry.translate(0, 0, 53 * UNIT_SCALE * 0.5 + 4.7 * UNIT_SCALE);

    var material = new THREE.MeshPhongMaterial({
      color: 0x00ff00
    });

    var epuck = new THREE.Mesh(geometry, material);

    var meshParent = new THREE.Group();
    /* Add all parts to a parent mesh */
    meshParent.add(epuck);

    /* Direction indicator */
    const dir_geometry = new THREE.CylinderGeometry( 0, 0.3, 1.5, 6 );
    dir_geometry.translate(0, 0, 86 * UNIT_SCALE + 0.4);
    dir_geometry.rotateZ(-Math.PI / 2);
    const dir_material = new THREE.MeshBasicMaterial( {color: 0xffff00} );
    const dir_cylinder = new THREE.Mesh( dir_geometry, dir_material );
    meshParent.add(dir_cylinder);

    const dir_edges = new THREE.EdgesGeometry( dir_geometry );
    const dir_line = new THREE.LineSegments( dir_edges, new THREE.LineBasicMaterial( { color: 0x000000 } ) );
    meshParent.add(dir_line);

    /* LEDs */
    var led_geometry = new THREE.TorusGeometry( 35 * UNIT_SCALE * 0.9, 
                                                0.2, 
                                                16, 
                                                100 );
    led_geometry.translate(0, 0, 86 * UNIT_SCALE * 0.9)
    var led_material = new THREE.MeshBasicMaterial( { color: 0x000000 } );
    var led = new THREE.Mesh( led_geometry, led_material );
    meshParent.add(led);

    /* Add Intersection Points */
    var pointsGeom = new THREE.BufferGeometry();
    pointsGeom.setAttribute('position', new THREE.BufferAttribute(
      new Float32Array(24 * 3), // 24 points * 3 axis per point
      3
    ));

    var points = new THREE.Points(pointsGeom, new THREE.PointsMaterial({
      color: 0x000000
    }));
    meshParent.add(points);

    /* Add lines for rays */
    for (let i = 0; i < 24; i++) {
      var lineGeom = new THREE.BufferGeometry();

      // attributes
      var linesPos = new Float32Array(2 * 3); //2 points per line * 3 axis per point
      lineGeom.setAttribute('position', new THREE.BufferAttribute(linesPos, 3));

      var line = new THREE.Line(lineGeom);

      meshParent.add(line);
      that.lines.push(line);
    }

    /* Update mesh parent */
    meshParent.position.x = entity.position.x * scale;
    meshParent.position.y = entity.position.y * scale;
    meshParent.position.z = entity.position.z * scale;

    that.mesh = meshParent;

    EntityLoadingFinishedFn(that);
  }

  getMesh() {
    return this.mesh;
  }

  update(entity) {
    var scale = this.scale

    if (this.mesh) {
      this.mesh.position.x = entity.position.x * scale;
      this.mesh.position.y = entity.position.y * scale;

      this.mesh.rotation.setFromQuaternion(new THREE.Quaternion(
        entity.orientation.x,
        entity.orientation.y,
        entity.orientation.z,
        entity.orientation.w));

      if (entity.leds) {
        /* Update LED colors */
        this.mesh.children[3].material.color.setHex(entity.leds[0]);
      }

      var pointMesh = this.mesh.children[13];

      if (entity.points.length > 0) {
        var points = pointMesh.geometry.getAttribute('position').array

        for (let i = 0; i < entity.points.length; i++) {
          var pointVals = entity.points[i].split(",")
          points[3 * i] = pointVals[0] * scale
          points[3 * i + 1] = pointVals[1] * scale
          points[3 * i + 2] = pointVals[2] * scale
        }
        pointMesh.geometry.getAttribute('position').needsUpdate = true;
      }

      /* Only draw given points, and hide all previous points */
      pointMesh.geometry.setDrawRange(0, entity.points.length);

      if (entity.rays.length > 0) {
        for (let i = 0; i < entity.rays.length; i++) {
          /*
              For each ray as a string,
              format -> "BoolIsChecked:Vec3StartPoint:Vec3EndPoint"
              For example -> "true:1,2,3:1,2,4"
          */

          var rayArr = entity.rays[i].split(":")
          var start = rayArr[1].split(",")
          var end = rayArr[2].split(",")

          var line = this.mesh.children[14 + i];
          if (line) {
            if (rayArr[0] == "true") {
              line.material.color.setHex(0xff00ff);
            } else {
              line.material.color.setHex(0x00ffff);
            }

            var positions = line.geometry.getAttribute('position').array

            positions[0] = start[0] * scale
            positions[1] = start[1] * scale
            positions[2] = start[2] * scale

            positions[3] = end[0] * scale
            positions[4] = end[1] * scale
            positions[5] = end[2] * scale

            line.geometry.getAttribute('position').needsUpdate = true;
            line.geometry.setDrawRange(0, 2);
          }
        }
      }

      /* Hide all the previous lines */
      /* 14 are the number of objects in meshParent before rays */
      for (let i = 14 + entity.rays.length; i < this.mesh.children.length; i++) {
        this.mesh.children[i].geometry.setDrawRange(0, 0);
      }
    }
  }
}