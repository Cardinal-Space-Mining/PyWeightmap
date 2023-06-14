from PyWeightMap import PyWeightMap as WeightMap

if __name__== "__main__":
    wm = WeightMap(8,9)
    print(wm)
    wm.add_border(2,50, WeightMap.border_top + WeightMap.border_bottom + WeightMap.border_left)
    print(WeightMap.maxWeight)
    print(WeightMap.minWeight)
    print(wm.max)
    wm.setWeight(0,0,169)
    print(wm.getWeight(0,0))
    print(wm)
    print(WeightMap.is_path_continuous(wm.getPath(0,0,0,wm.height-1)))
    print(WeightMap.compressPath(wm.getPath(0,0,0,wm.height-1)))

