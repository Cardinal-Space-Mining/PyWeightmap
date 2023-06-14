src_files = ./map_util.cpp ./WeightMap.cpp ./PyWeightMap.cpp

pyBind11Include := $(shell python3 -m pybind11 --includes)
extSuffix = $(shell python3-config --extension-suffix)


make:
	g++ -O3 -flto -Wall -shared -std=c++14 -Wall -Wextra -s -fPIC $(pyBind11Include) $(src_files) -o PyWeightMap$(extSuffix)

test:
	python3 ./test.py

example:
	g++ -O3 -Wall -shared -std=c++11 -fPIC $(pyBind11Include) example.cpp -o example$(extSuffix)

clean:
	rm ./*.o 
	rm ./*.so