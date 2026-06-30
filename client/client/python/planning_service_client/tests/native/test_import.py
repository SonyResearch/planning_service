import unittest


class TestNativeImport(unittest.TestCase):
    def test_native_import(self):
        import planning_service_client.native as native

        self.assertTrue(hasattr(native, "types"))
        self.assertTrue(hasattr(native, "visualizer"))

        self.assertTrue(hasattr(native.visualizer, "VisualizerClient"))
        self.assertTrue(hasattr(native.types, "Conf"))
        self.assertTrue(hasattr(native.types, "SystemConf"))
        self.assertTrue(hasattr(native.types, "FrameRelativePose"))
        self.assertTrue(hasattr(native.types, "ContextId"))
        self.assertTrue(hasattr(native.types, "Rgba"))
        self.assertTrue(hasattr(native.types, "Shape"))
        self.assertTrue(hasattr(native.types, "ShapeInFrame"))
        self.assertTrue(hasattr(native.types, "Sphere"))
        self.assertTrue(hasattr(native.types, "Cylinder"))
        self.assertTrue(hasattr(native.types, "Capsule"))
        self.assertTrue(hasattr(native.types, "Box"))


if __name__ == "__main__":
    unittest.main()
