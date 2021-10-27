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

        var that = this;
        var geometry = new THREE.BoxBufferGeometry(
            entity.scale.x * scale,
            entity.scale.y * scale,
            0.3
        );

        /* Bring above ground */
        geometry.translate(0, 0, entity.scale.z * scale * 0.5);

        var demand = entity.task.demand;

        var color = null;
        if (demand == 0) {
            color = 0x00ff00;
        } else {
            color = 0xff0000;
        }

        var material = new THREE.MeshPhongMaterial({
            color: color,
            transparent: true,
            opacity: 0.5,
        });

        this.task = new THREE.Mesh(geometry, material);

        this.task.rotation.setFromQuaternion(new THREE.Quaternion(
            entity.orientation.x,
            entity.orientation.y,
            entity.orientation.z,
            entity.orientation.w));

        var meshParent = new THREE.Group();
        /* Add all parts to a parent mesh */
        meshParent.add(this.task);

        this.sprite = new THREE.TextSprite({
            alignment: 'center',
            color: '#000000',
            fontFamily: '"Times New Roman", Times, serif',
            fontSize: 8,
            text: [
                entity.task.demand,
            ].join('\n'),
        });

        this.sprite.position.z = 10;

        meshParent.add(this.sprite);

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

        if(this.mesh) {

            /* Delete existing child objects */
            for (let i = 0; i < this.mesh.children.length; i++) {
                this.mesh.remove(this.mesh.children[1]);
            }

            /* Add the child objects back */
            this.mesh.add(this.task);

            this.sprite.text = [
                entity.task.demand,
            ].join('\n');
            this.mesh.add(this.sprite);

            /* Update color of task */
            var color = null;
            if (entity.task.demand == 0) {
                color = 0x00ff00;
            } else {
                color = 0xff0000;
            }
    
            this.mesh.children[0].material.color.setHex(color);
        }
    }
}