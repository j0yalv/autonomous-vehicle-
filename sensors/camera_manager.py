import carla
import cv2
import numpy as np


class CameraManager:

    def __init__(self, world, vehicle):

        blueprint_library = world.get_blueprint_library()

        camera_bp = blueprint_library.find(
            'sensor.camera.rgb'
        )

        camera_bp.set_attribute('image_size_x', '640')
        camera_bp.set_attribute('image_size_y', '360')
        camera_bp.set_attribute('fov', '110')

        camera_transform = carla.Transform(
            carla.Location(x=-6, z=3)
        )

        self.camera = world.spawn_actor(
            camera_bp,
            camera_transform,
            attach_to=vehicle
        )

        self.camera.listen(self.process_image)

    def process_image(self, image):

        array = np.frombuffer(
            image.raw_data,
            dtype=np.uint8
        )

        array = array.reshape(
            (image.height, image.width, 4)
        )

        array = array[:, :, :3]

        cv2.imshow("CARLA Camera", array)

        cv2.waitKey(1)

    def destroy(self):

        self.camera.destroy()

        cv2.destroyAllWindows()