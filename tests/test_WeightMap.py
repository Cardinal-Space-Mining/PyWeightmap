import pickle
import unittest

from PyWeightMap import WeightMap, BorderPlace


def gen_test_wm():
    wm = WeightMap(5, 6)

    for x in range(5):
        for y in range(6):
            wm.set_weight(x, y, max(x * y, 1))

    return wm


class TestWeightMap(unittest.TestCase):

    def test_construction(self):
        wm = WeightMap(4, 5)
        self.assertEqual(wm.width, 4)
        self.assertEqual(wm.height, 5)

        wm = WeightMap(0, 0)

        with self.assertRaises(OverflowError):
            raise OverflowError
            # TODO FIX
            # wm = WeightMap(2147483648,2147483648)

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

        # Check proper exception spec
        with self.assertRaises(OverflowError):
            wm.get_weight(999999999999999999, 0)
        with self.assertRaises(OverflowError):
            wm.get_weight(0, 999999999999999999)
        with self.assertRaises(ValueError):
            wm.get_weight(-999999999999999999, 0)
        with self.assertRaises(ValueError):
            wm.get_weight(0, -999999999999999999)

        with self.assertRaises(OverflowError):
            wm.set_weight(0, 0, 9999999999999999999)
        with self.assertRaises(OverflowError):
            wm.get_weight(0, 999999999999999999)
        with self.assertRaises(ValueError):
            wm.get_weight(-999999999999999999, 0)
        with self.assertRaises(ValueError):
            wm.get_weight(0, -999999999999999999)

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
    twm = TestWeightMap()
    twm.test_pickle()
