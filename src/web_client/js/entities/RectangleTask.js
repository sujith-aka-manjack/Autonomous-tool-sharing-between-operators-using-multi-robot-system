/**
 * @file <client/js/entities/RectangleTask.js>
 * 
 * @author Genki Miyauchi - <g.miyauchi@sheffield.ac.uk>
 * 
 */

 class RectangleTask {
    constructor(entity, scale, EntityLoadingFinishedFn) {
        this.scale = scale;
        this.entity = entity;

        var geometry = new THREE.BoxBufferGeometry(
            entity.scale.x * scale,
            entity.scale.y * scale,
            0.3
        );

        /* Bring above ground */
        geometry.translate(0, 0, entity.scale.z * scale * 0.5);

        var color = null;
        if (entity.is_movable) {
            color = 0xff0000;
        } else {
            color = 0xff0000;
        }

        var material = new THREE.MeshPhongMaterial({
            color: color,
            transparent: true,
            opacity: 0.5,
        });

        var task = new THREE.Mesh(geometry, material);

        task.rotation.setFromQuaternion(new THREE.Quaternion(
            entity.orientation.x,
            entity.orientation.y,
            entity.orientation.z,
            entity.orientation.w));
        task.position.x = entity.position.x * scale;
        task.position.y = entity.position.y * scale;
        task.position.z = entity.position.z * scale;

        this.mesh = task;

        EntityLoadingFinishedFn(this);
    }

    getMesh() {
        return this.mesh;
    }

    update(entity) {
        if (entity.is_movable) {
            try {
                this.mesh.position.x = entity.position.x * this.scale;
                this.mesh.position.y = entity.position.y * this.scale;

                this.mesh.rotation.setFromQuaternion(new THREE.Quaternion(
                    entity.orientation.x,
                    entity.orientation.y,
                    entity.orientation.z,
                    entity.orientation.w));
            } catch (ignored) { }
        }
    }
}