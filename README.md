# PyWeightmap
Provides bindings to the C++ WeightMap class.
* enum `BorderPlace`:
  * TOP
  * BOTTOM
  * RIGHT
  * LEFT
* function `compress_path`
* function `is_path_continuous`
* function `smooth_path`
* class `WeightMap`
  * methods:
    * get_weight
    * set_weight
    * get_path
    * path_to_x
    * add_border
    * add_obstical
    * is_valid_point
    * get_weights
    * from_bytes
    * __bytes__
    * __str__
    * __setstate__
    * __getstate__
  * properties:
    * width
    * height
    * max_value
  * static properties:
    * max_weight
    * min_weight

# Installation
1. Download source code
2. Navigate to top level.
3. Run `pip install ./`

# Documentation
1. Download source code.
2. Navigate to the docs directory
3. Run `make html`
4. Open `./docs/_build/html/PyWeightMap.html`
