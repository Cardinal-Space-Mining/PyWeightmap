#pragma once
// STL includes
#include <ostream>	  //std::ostream
#include <stdint.h>	  //uint16_t, uint_fast16_t
#include <string>	  //std::string
#include <vector>	  //std::vector
#include <utility>	  //std::pair
#include <queue>	  //std::queue
#include <functional> //std::reference_wrapper

// My Includes
#include "BlockArray2D.hpp" // BlockArray2DRT

// Weight Distribution

enum BorderPlace: int{
	TOP = 1 << 0,
	BOTTOM = 1 << 1,
	RIGHT = 1 <<2,
	LEFT = 1 << 3,
	UNKNOWN = 1 << 4
};

bool contains(const BorderPlace &self, const BorderPlace &other);
bool isValid(const BorderPlace &self);
const char *to_string(const BorderPlace &self);

inline BorderPlace operator|(BorderPlace a, BorderPlace b)
{
    return static_cast<BorderPlace>(static_cast<int>(a) | static_cast<int>(b));
}


using mapsize_t = uint16_t;

using fast_mapsize_t = uint_fast16_t;

using weight_t = uint16_t;

using fweight_t = float;


/// <summary>
/// A dense weighted map for navigation purposes.
/// Memory is runtime allocated
/// </summary>
class WeightMap
{
private:

	// Inner Class/Method Declarations
	enum class Color : mapsize_t
	{
		BLACK = 0,
		RED = 1,
		GREEN = 2,
		UNKNOWN = 3
	};

	friend std::ostream &operator<<(std::ostream &os, const Color &c);

	typedef struct Node
	{
		const mapsize_t x;
		const mapsize_t y;
		mapsize_t parent_x;
		mapsize_t parent_y;
		Color color;
		weight_t weight;
		fweight_t costFromSrc;

		Node(const mapsize_t x,
			const mapsize_t y,
			const mapsize_t parent_x,
			const mapsize_t parent_y,
			const Color color,
			const weight_t weight,
			const fweight_t costFromSrc);
		Node();

		bool operator==(const Node &other);
	} Node;

	friend std::ostream &operator<<(std::ostream &os, const Node &n);

	class NodeCmp
	{
	public:
		bool operator()(const Node &a, const Node &b);
		bool operator()(const Node *a, const Node *b);
	};


	struct DijkstrasMove
	{
		int8_t dx;
		int8_t dy;
		fweight_t weight_multiplier;
	};

	static constexpr fweight_t SQRT_2 = 1.42f; // Slightly more than actual sqrt2
	static constexpr WeightMap::DijkstrasMove moves[] = {
		{0, 1, 1},
		{-1, 0, 1},
		{0, -1, 1},
		{1, 0, 1},
		{1, 1, SQRT_2},
		{-1, 1, SQRT_2},
		{-1, -1, SQRT_2},
		{1, -1, SQRT_2}};
	static constexpr size_t numMoves = sizeof(moves) / sizeof(moves[0]);


public:
	using point_t = std::pair<mapsize_t, mapsize_t>;
	using path_t = std::vector<WeightMap::point_t>;

	// Constructors/Destructors
	WeightMap(mapsize_t width, mapsize_t height);

	~WeightMap() = default;

	// Accessors
	inline constexpr mapsize_t getWidth() const noexcept
	{
		return width;
	}

	inline constexpr mapsize_t getHeight() const noexcept
	{
		return height;
	}

	inline static constexpr weight_t getMaxWeight() noexcept
	{
		return 255;
	}

	inline static constexpr weight_t getMinWeight() noexcept
	{
		return 1;
	}

	weight_t getMaxWeightInMap() const;

	weight_t getWeight(mapsize_t x, mapsize_t y) const;

	void setWeight(mapsize_t x, mapsize_t y, weight_t weight);

	std::string to_string(bool extraData = false) const;

	friend std::ostream &operator<<(std::ostream &os, const WeightMap &wm);

	// Path Planning
	path_t pathToXVal(mapsize_t srcX, mapsize_t srcY, mapsize_t dstX);

	path_t getPath(mapsize_t srcX, mapsize_t srcY, mapsize_t dstX, mapsize_t dstY);

	static void compressPath(path_t &path);

	static void smoothPath(path_t& path, fweight_t allowed_ratio);

	void addBoarder(mapsize_t boarder_width, weight_t boarder_weight, BorderPlace place);

	void addObstical(mapsize_t x_in, mapsize_t y_in, mapsize_t radius, weight_t weight, bool gradiant);

	WeightMap(WeightMap &) = default;

	WeightMap() = delete;

	auto operator=(WeightMap &) = delete;

	WeightMap(WeightMap &&) = default;

	bool isValidPoint(int32_t x, int32_t y) const;

	static inline bool isValidWeight(weight_t weight)
	{
		return weight <= WeightMap::getMaxWeight() && weight >= WeightMap::getMinWeight();
	}

	weight_t* getWeights() const;

	static std::string path_to_str(path_t &path);

	static bool is_path_continuous(path_t &path);

	static std::string point_to_string(const point_t &pt);

	std::pair<const char*, const size_t> serialize() const;

	static WeightMap deserialize(std::pair<const char*, const size_t> bytes);

	bool operator==(const WeightMap& other) const;

	size_t hash() const;

private:
	// Helper Functions
	using queue_t = std::priority_queue<Node *, std::vector<Node *>, NodeCmp>;

	void resetMap();

	void dijkstrasMakeMoves(const DijkstrasMove *moves, const size_t numMoves, const Node &currentNode, queue_t &q);

	static fweight_t distance(fweight_t x1, fweight_t y1, fweight_t x2, fweight_t y2);

	path_t generate_path(const Node &src, const Node &dst) const;

	static fweight_t length_ratio(point_t& l, point_t& m, point_t& r);

	fweight_t get_linear_cost(Node& a, Node& b);

	void theta_smooth_path(path_t& p);

private:
	// Class Fields
	const mapsize_t width;
	const mapsize_t height;

	//Indexed in x-y form
	BlockArray2DRT<Node> arr;
};

template <>
struct std::hash<WeightMap>
{
  std::size_t inline operator()(const WeightMap& k) const
  {
    return k.hash();
  }
};