from PyWeightMap import WeightMap, BorderPlace
import unittest
import pickle

class TestWeightMap(unittest.TestCase):

    def test_construction(self):
        wm = WeightMap(4,5)
        self.assertEqual(wm.width, 4)
        self.assertEqual(wm.height, 5)

        wm = WeightMap(0,0)

        with self.assertRaises(OverflowError):
            wm = WeightMap(2147483648,2147483648)

    def test_set_get_weight(self):
        # Test the underlying value is being set
        wm = WeightMap(10,11)
        test_weight: int = 10
        wm.set_weight(0,0,test_weight)
        self.assertEqual(test_weight, wm.get_weight(0,0))
        
        # Check proper exception spec
        with self.assertRaises(OverflowError):
            wm.get_weight(999999999999999999,0)
        with self.assertRaises(OverflowError):
            wm.get_weight(0,999999999999999999)
        with self.assertRaises(ValueError):
            wm.get_weight(-999999999999999999,0)
        with self.assertRaises(ValueError):
            wm.get_weight(0,-999999999999999999)

        with self.assertRaises(OverflowError):
            wm.set_weight(0,0,9999999999999999999)
        with self.assertRaises(OverflowError):
            wm.get_weight(0,999999999999999999)
        with self.assertRaises(ValueError):
            wm.get_weight(-999999999999999999,0)
        with self.assertRaises(ValueError):
            wm.get_weight(0,-999999999999999999)

    def test_pickle(self):
        
        wm = WeightMap(10,11)
        wm.add_border(2,25, BorderPlace.TOP)
        

        bts = pickle.dumps(wm)
        
        
        wm2 = pickle.loads(bts)
 
        for x in range(0, wm.width):
            for y in range(0, wm.height):
                self.assertEqual(wm.get_weight(x,y), wm2.get_weight(x,y))
            


    def test_add_border(self):
        wm = WeightMap(10,10)
        wm.add_border(2,25, BorderPlace.BOTTOM | BorderPlace.TOP)


if __name__ == '__main__':
    twm = TestWeightMap()
    twm.test_pickle()

