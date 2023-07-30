import pickle
import unittest
import numpy

from PyWeightMap import WeightMap, BorderPlace


def gen_test_wm():
    wm = WeightMap(5, 6)

    for x in range(5):
        for y in range(6):
            wm.set_weight(x, y, max(x * y, 1))

    return wm


class TestWeightMap(unittest.TestCase):

    def test_add_obsticle(self):
        wm = WeightMap(5,5)
        wm.add_obstical(2,2,2,5, False)
        array1 = numpy.array([  [1, 1, 5, 1, 1],
                                [1, 5, 5, 5, 1],
                                [5, 5, 5, 5, 5],
                                [1, 5, 5, 5, 1],
                                [1, 1, 5, 1, 1]], dtype=numpy.uint16)

        self.assertTrue(numpy.array_equal(wm.get_weights(), array1))

        wm = WeightMap(10,10)
        wm.add_obstical(5,5,5,255,True)
        array2 = numpy.array([  [  1,   1,   1,   1,   1,   1,   1,   1,   1,   1],
                                [  1,   1,   1,  26,  44,  50,  44,  26,   1,   1],
                                [  1,   1,  38,  71,  93, 101,  93,  71,  38,   1],
                                [  1,  26,  71, 110, 140, 152, 140, 110,  71,  26],
                                [  1,  44,  93, 140, 182, 203, 182, 140,  93,  44],
                                [  1,  50, 101, 152, 203, 255, 203, 152, 101,  50],
                                [  1,  44,  93, 140, 182, 203, 182, 140,  93,  44],
                                [  1,  26,  71, 110, 140, 152, 140, 110,  71,  26],
                                [  1,   1,  38,  71,  93, 101,  93,  71,  38,   1],
                                [  1,   1,   1,  26,  44,  50,  44,  26,   1,   1]], dtype=numpy.uint16)
        self.assertTrue(numpy.array_equal(wm.get_weights(), array2))

        wm.add_obstical(0,0,2,8, False)

        wm.add_obstical(5,5,2,2, False)

    def test_construction(self):
        wm = WeightMap(4, 5)
        self.assertEqual(wm.width, 4)
        self.assertEqual(wm.height, 5)
        self.assertRaises(ValueError, lambda: WeightMap(0, 0))

    def test_get_weights(self):
        wm = gen_test_wm()
        wm_weights = wm.get_weights()
        self.assertEqual(wm_weights.shape, (wm.width, wm.height))

        for x in range(wm.width):
            for y in range(wm.height):
                self.assertEqual(wm.get_weight(x, y), wm_weights[x][y])

    def test_set_get_weight(self):
        # Test the underlying value is being set
        wm = WeightMap(10, 11)
        test_weight: int = 10
        wm.set_weight(0, 0, test_weight)
        self.assertEqual(test_weight, wm.get_weight(0, 0))

        wm.set_weight(0, 0, 1)

        for x in range(wm.width):
            for y in range(wm.height):
                self.assertEqual(wm.get_weight(x, y), 1)

    def test_pickle(self):

        wm = gen_test_wm()

        bts = pickle.dumps(wm)

        wm2 = pickle.loads(bts)

        for x in range(0, wm.width):
            for y in range(0, wm.height):
                self.assertEqual(wm.get_weight(x, y), wm2.get_weight(x, y))

    def test_add_border(self):
        wm = WeightMap(10, 10)
        wm.add_border(2, 25, BorderPlace.BOTTOM | BorderPlace.TOP)

    def test_hashing(self):
        wm1 = WeightMap(10,10)
        wm2 = WeightMap(10,10)
        self.assertEqual(wm1, wm2)
        self.assertEqual(hash(wm1), hash(wm2))
        wm2.set_weight(0,0,12)
        self.assertNotEqual(wm1, wm2)
        self.assertNotEqual(hash(wm1), hash(wm2))



if __name__ == '__main__':
    unittest.main()
    
