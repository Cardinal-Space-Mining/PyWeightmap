// Project Includes
#include "WeightMap.hpp"

#include <limits>	 //std::numeric_limits
#include <algorithm> //std::min
#include <stdexcept> //std::out_of_range, std::invalid_argument
#include <sstream>	 //std::stringstream
#include <iomanip>	 //std::setw
#include <algorithm> //std::reverse
#include <cmath>	 //std::sqrt
#include <array>
#include <functional>
#include <cassert>



// Node Constructor
WeightMap::Node::Node(const mapsize_t x_in,
		   const mapsize_t y_in,
		   const mapsize_t parent_x_in,
		   const mapsize_t parent_y_in,
		   const Color color_in,
		   const weight_t weight_in,
		   const fweight_t costFromSrc_in) : x(x_in), y(y_in), parent_x(parent_x_in), parent_y(parent_y_in),
											 color(color_in), weight(weight_in), costFromSrc(costFromSrc_in){};

WeightMap::Node::Node(): x(0), y(0), parent_x(0), parent_y(0), color(Color::BLACK), weight(0), costFromSrc(0){};

//---------------	Boarder Place Methods	----------------------------

bool contains(const BorderPlace &self, const BorderPlace &other){
	return (static_cast<int>(self) & static_cast<int>(other)) == static_cast<int>(other);
}
bool isValid(const BorderPlace &self)
{
	return static_cast<int>(self) < static_cast<int>(BorderPlace::UNKNOWN);
}

const char *to_string(const BorderPlace &self){
	if (self == BorderPlace::TOP)
	{
		return "TOP";
	}
	if (self == BorderPlace::BOTTOM)
	{
		return "BOTTOM";
	}
	if (self == BorderPlace::RIGHT)
	{
		return "RIGHT";
	}
	if (self == BorderPlace::LEFT)
	{
		return "LEFT";
	}

	if (static_cast<int>(self) >= static_cast<int>(BorderPlace::UNKNOWN))
	{
		return "UNKNOWN";
	}

	return "COMBO";

}

//---------------	WeightMap Methods	----------------------------
WeightMap::WeightMap(mapsize_t width, mapsize_t height) : width(width), height(height), arr(width, height)
{
	// Populate nodes
	for (fast_mapsize_t x = 0; x < width; x++)
	{
		for (fast_mapsize_t y = 0; y < height; y++)
		{
			new (&arr[x][y]) Node(x, y, std::numeric_limits<mapsize_t>::max(),
								  std::numeric_limits<mapsize_t>::max(),
								  Color::BLACK,
								  1,
								  std::numeric_limits<decltype(Node::costFromSrc)>::infinity());
		}
	}
}

weight_t* WeightMap::getWeights() const{	
	weight_t* const buff = new weight_t[this->width * this->height];

	for (size_t y = 0; y < this->height; y++){
		for (size_t x = 0; x <  this-> width; x++){
			buff[(y * this->width) + x] = arr[x][y].weight;
		}
	}

	return buff;
}

bool WeightMap::operator==(const WeightMap & other) const
{
	if(this->width != other.width || this->height != other.height){
		return false;
	}
	for (size_t x = 0; x < this->width; x++){
		for (size_t y = 0; y < this->height; y++){
			if (this->arr[x][y].weight != other.arr[x][y].weight){
				return false;
			}
		}
	}

	return true;
}

// Based off an implementation of Arrays.deepHashCode(e) from Java
size_t WeightMap::hash() const{
	constexpr const size_t P = 97;

	const std::hash<mapsize_t> map_hashf;
	const std::hash<weight_t> weight_hashf;

	size_t hash = map_hashf(this->width) +  (map_hashf(this->height) * P);
	
	for (size_t x = 0; x < this->width; x++){
		for (size_t y =0; y< this->height; y++){
			const size_t element_hash = weight_hashf(this->arr[x][y].weight);
			hash = (P * hash) + element_hash;
		}
	}
	return hash;
}

weight_t WeightMap::getMaxWeightInMap() const
{
	weight_t curr_max = std::numeric_limits<weight_t>::min();

	for (fast_mapsize_t x = 0; x < this->width; x++)
	{
		for (fast_mapsize_t y = 0; y < this->height; y++)
		{
			curr_max = std::max(arr[x][y].weight, curr_max);
		}
	}
	return curr_max;
}

weight_t WeightMap::getWeight(mapsize_t x, mapsize_t y) const
{
	if (!isValidPoint(x, y))
	{
		char error_message[60];
		std::snprintf(error_message, sizeof(error_message), "Point (%u, %u) out of bounds!", x, y);
		throw std::invalid_argument(std::string(error_message));
	}

	return arr[x][y].weight;
}

void WeightMap::setWeight(mapsize_t x, mapsize_t y, weight_t weight)
{
	if (!isValidPoint(x, y))
	{
		char error_message[60];
		std::snprintf(error_message, sizeof(error_message), "Point (%u, %u) out of bounds!", x, y);
		throw std::invalid_argument(std::string(error_message));
	}
	if (weight > this->getMaxWeight())
	{
		char error_message[60];
		std::snprintf(error_message, sizeof(error_message), "Weight %u is greater than the max weight %u!", weight, this->getMaxWeight());
		throw std::invalid_argument(std::string(error_message));
	}

	arr[x][y].weight = weight;
}

std::string WeightMap::to_string(bool extraData) const
{

	std::stringstream ss;

	if (extraData)
	{
		ss << *this;
	}
	else
	{

		const size_t max_width = std::to_string((int)this->getMaxWeightInMap()).size();

		for (fast_mapsize_t y = 0; y < height; y++)
		{
			for (fast_mapsize_t x = 0; x < width; x++)
			{
				ss << std::setw(max_width) << std::setfill(' ') << arr[x][y].weight << ' ';
			}
			ss << '\n';
		}
	}

	return ss.str();
}


WeightMap::path_t WeightMap::pathToXVal(mapsize_t srcX, mapsize_t srcY, mapsize_t dstX)
{

	if (!isValidPoint(srcX, srcY))
	{
		char error_message[60];
		std::snprintf(error_message, sizeof(error_message), "Source Point (%u, %u) out of bounds!", srcX, srcY);
		throw std::invalid_argument(std::string(error_message));
	}

	if (dstX >= this->width)
	{
		char error_message[60];
		std::snprintf(error_message, sizeof(error_message), "X value to travel to {%u} out of bounds!", dstX);
		throw std::invalid_argument(std::string(error_message));
	}

	Node &src = this->arr[srcX][srcY];

	this->resetMap();

	src.parent_x = std::numeric_limits<mapsize_t>::max();
	src.parent_y = std::numeric_limits<mapsize_t>::max();
	src.color = Color::RED;
	src.costFromSrc = 0;

	std::vector<Node *> queue_backer;
	queue_backer.reserve(
		static_cast<size_t>(this->width) * static_cast<size_t>(this->height));
	WeightMap::queue_t q(NodeCmp(), std::move(queue_backer));
	q.push(&src);
	while (!q.empty())
	{
		// Taking a reference from a queue and destroying it is ok because the Node does not live in the queue,
		//  Rather, the queue holds references to a Node held by the weightmap, and we are copying out that reference
		//  And then removing the reference object from the queue
		Node &currentNode = *q.top();
		q.pop();

		dijkstrasMakeMoves(moves, numMoves, currentNode, q);

		if (dstX <= currentNode.x)
		{
			return generate_path(src, currentNode);
		}

		currentNode.color = Color::GREEN;
	}

	// We should not reach here. The if statement in the while loop should have triggered on the first node with the propper x-value
	assert(false);
}

WeightMap::path_t WeightMap::generate_path(const Node &src, const Node &dst) const
{
	path_t path;

	const Node *currentNode = &dst;
	while (isValidPoint(currentNode->parent_x, currentNode->parent_y))
	{
		path.emplace_back(currentNode->x, currentNode->y);
		currentNode = &this->arr[currentNode->parent_x][currentNode->parent_y];
	}
	assert(path.size() <= (this->height * this->width));
	path.emplace_back(currentNode->x, currentNode->y);
	std::reverse(path.begin(), path.end());
	assert(path[0].first == src.x && path[0].second == src.y);
	return path;
}

WeightMap::path_t WeightMap::getPath(mapsize_t srcX, mapsize_t srcY, mapsize_t dstX, mapsize_t dstY)
{
	if (!isValidPoint(srcX, srcY))
	{
		char error_message[60];
		std::snprintf(error_message, sizeof(error_message), "Source Point (%u, %u) out of bounds!", srcX, srcY);
		throw std::invalid_argument(std::string(error_message));
	}
	if (!isValidPoint(dstX, dstY))
	{
		char error_message[60];
		std::snprintf(error_message, sizeof(error_message), "Destination Point (%u, %u) out of bounds!", dstX, dstY);
		throw std::invalid_argument(std::string(error_message));
	}

	if (srcX == dstX && srcY == dstY)
	{
		return {{srcX, srcY}};
	}

	Node &src = this->arr[srcX][srcY];
	Node &dst = this->arr[dstX][dstY];

	this->resetMap();

	src.parent_x = std::numeric_limits<mapsize_t>::max();
	src.parent_y = std::numeric_limits<mapsize_t>::max();
	src.color = Color::RED;
	src.costFromSrc = 0;

	std::vector<Node *> queue_backer;
	queue_backer.reserve(this->width * this->height);
	WeightMap::queue_t q(NodeCmp(), std::move(queue_backer));
	q.push(&src);
	while (!q.empty())
	{
		Node &currentNode = *q.top();
		q.pop();

		if (currentNode.x == dst.x && currentNode.y == dst.y)
		{
			break;
		}

		dijkstrasMakeMoves(moves, numMoves, currentNode, q);

		currentNode.color = Color::GREEN;
	}

	// Generate Path

	return generate_path(src, dst);
}

/*
O(N) runtime, O(1) memory useage
*/
void WeightMap::compressPath(path_t &path)
{
	if (path.size() < 3)
	{
		return;
	}

	size_t book_mark = 1;

	for (fast_mapsize_t idx = 1; idx < path.size()-1; idx++)
	{
		const int8_t b_dx = path[idx].first - path[idx - 1].first;
		const int8_t b_dy = path[idx].second - path[idx - 1].second;
		const int8_t f_dx = path[idx+1].first - path[idx].first;
		const int8_t f_dy = path[idx+1].second - path[idx].second;
		if (b_dx != f_dx || f_dy != b_dy)
		{
			path[book_mark] = path[idx];
			book_mark++;
		}
	}
	path[book_mark] = path[path.size() - 1];

	path.resize(book_mark + 1);
}

void WeightMap::smoothPath(WeightMap::path_t &path, fweight_t allowed_ratio)
{
	size_t start_size;
	do{
		start_size = path.size();
		for (size_t i = 1; i < path.size() - 1; i++)
		{
			const fweight_t ratio = WeightMap::length_ratio(path[i - 1], path[i], path[i + 1]);
			if (ratio > allowed_ratio)
			{
				path.erase(path.begin() + (path_t::difference_type)i);
			}
		}
	}while (start_size - path.size() != 0);
}


void WeightMap::theta_smooth_path(WeightMap::path_t &path)
{
	size_t start_size;
	do{
		start_size =  path.size();
		for (size_t i = 1; i < path.size() - 1; i++)
		{
			Node &left_node = arr[path[i - 1].first][path[i - 1].second];
			Node &right_node = arr[path[i + 1].first][path[i + 1].second];
			const float direct_cost = this->get_linear_cost(left_node, right_node);
			const float new_cost = direct_cost + left_node.costFromSrc;
			if (new_cost < right_node.costFromSrc)
			{
				//Remove Node B from path
				path.erase(path.begin() + i);

				//Update costs from source to reflect change
				const float cost_difference = right_node.costFromSrc - new_cost;
				for (size_t a = i; a < path.size(); a++)
				{
					arr[path[a].first][path[a].second].costFromSrc -= cost_difference;
				}
				break;
			}
		}
	}while (start_size - path.size() != 0);
}

void WeightMap::addBoarder(mapsize_t boarder_width, weight_t boarder_weight, BorderPlace place)
{

	if (!WeightMap::isValidWeight(boarder_weight))
	{
		char error_message[60];
		std::snprintf(error_message, sizeof(error_message), "Weight {%u} out of bounds!", boarder_weight);
		throw std::invalid_argument(std::string(error_message));
	}

	if (!isValid(place))
	{

		char error_message[60];
		std::snprintf(error_message, sizeof(error_message), "Place {0x%x} is invalid!", static_cast<int>(place));
		throw std::invalid_argument(std::string(error_message));
	}

	if ((contains(place, BorderPlace::TOP) || contains(place, BorderPlace::BOTTOM)) && boarder_width > height)
	{
		throw std::invalid_argument("Boarder Width is greater than the height of the board");
	}

	if ((contains(place, BorderPlace::RIGHT) || contains(place, BorderPlace::LEFT)) && boarder_width > width)
	{
		throw std::invalid_argument("Boarder Width is greater than the widtg of the board");
	}

	if (contains(place, BorderPlace::TOP))
	{
		for (fast_mapsize_t y = 0; y < boarder_width; y++)
		{
			for (fast_mapsize_t x = 0; x < width; x++)
			{
				if (isValidPoint(x, y))
				{
					arr[x][y].weight = boarder_weight;
				}
			}
		}
	}

	if (contains(place, BorderPlace::BOTTOM))
	{
		for (fast_mapsize_t y = height - boarder_width; y < height; y++)
		{
			for (fast_mapsize_t x = 0; x < width; x++)
			{
				arr[x][y].weight = boarder_weight;
			}
		}
	}

	if (contains(place, BorderPlace::RIGHT))
	{
		for (fast_mapsize_t x = width - boarder_width; x < width; x++)
		{
			for (fast_mapsize_t y = 0; y < height; y++)
			{
				arr[x][y].weight = boarder_weight;
			}
		}
	}

	if (contains(place, BorderPlace::LEFT))
	{
		for (fast_mapsize_t x = 0; x < boarder_width; x++)
		{
			for (fast_mapsize_t y = 0; y < height; y++)
			{
				arr[x][y].weight = boarder_weight;
			}
		}
	}
}


void WeightMap::addObstical(mapsize_t x_in, mapsize_t y_in, mapsize_t radius, weight_t weight, bool gradient)
{
	if (!WeightMap::isValidWeight(weight))
	{
		char error_message[60];
		std::snprintf(error_message, sizeof(error_message), "Weight {%u} out of bounds!", weight);
		throw std::invalid_argument(std::string(error_message));
	}

	const fweight_t radius_f = radius; // Don't want to convert it to a float each time
	for (size_t dy = radius; dy > 0; dy--)
	{
		for (size_t dx = 0; dx <= radius; dx++)
		{
			if (dx > dy)
			{
				// We have reached the x-y symmetry line
				// The values on the other side of that line are already being
				// filled by symmetry rules.
				// We can break early
				break;
			}

			const fweight_t dist = std::sqrt((dx * dx) + (dy * dy));
			if (dist > radius_f)
			{
				// We have reached the edge of this line, and all values after should remain unchanged
				// Cancel x-iteration early
				break;
			}

			const weight_t calculated_weight = gradient ? (1.0 - (dist / radius_f)) * weight : weight;

			//Calculate points
			using point = std::pair<size_t, size_t>;

			point points[8] = {
				{x_in + dx, y_in + dy}, // Standard Location
				{x_in - dx, y_in + dy}, // Symmetry accross x-axis
				{x_in + dx, y_in - dy}, // Symmetry accross y-axis
				{x_in - dx, y_in - dy}, // Symmetry accross x = -y

				//These points are the above, but reflected across y=x
				{x_in + dy, y_in + dx}, // Standard Location
				{x_in - dy, y_in + dx}, // Symmetry accross x-axis
				{x_in + dy, y_in - dx}, // Symmetry accross y-axis
				{x_in - dy, y_in - dx}, // Symmetry accross x = -y
			};

			for (const auto& pt: points){
				if (isValidPoint(pt.first, pt.second)){ //Bounds check because people can add objects at the edge of the map
					arr[pt.first][pt.second].weight = std::max(calculated_weight, arr[pt.first][pt.second].weight);
				}
			}
		}
	}

	arr[x_in][y_in].weight = std::max(weight, arr[x_in][y_in].weight); // Standard Location
}

bool WeightMap::isValidPoint(int32_t x, int32_t y) const
{
	return (x < this->width && y < this->height) && (x >= 0 && y >= 0);
}

std::string WeightMap::path_to_str(path_t &path)
{
	std::stringstream ss;
	for (auto &point : path)
	{
		ss << '(' << point.first << ", " << point.second << ")\n";
	}
	return ss.str();
}

bool WeightMap::is_path_continuous(path_t &path)
{
	const fweight_t max_dist = std::sqrt(2);
	for (size_t i = 1; i < path.size(); i++)
	{
		point_t &pt1 = path[i - 1];
		point_t &pt2 = path[i];
		const fweight_t dist = WeightMap::distance(pt1.first, pt1.second, pt2.first, pt2.second);
		if (dist > max_dist)
		{
			return false;
		}
	}
	return true;
}

std::string WeightMap::point_to_string(const point_t &pt)
{
	std::stringstream ss;
	ss << '(' << pt.first << ", " << pt.second << ')';
	return ss.str();
}

// Helper Functions
void WeightMap::resetMap()
{
	for (fast_mapsize_t x = 0; x < width; x++)
	{
		for (fast_mapsize_t y = 0; y < height; y++)
		{
			// Set up default values
			Node &current_node = arr[x][y];
			current_node.parent_x = std::numeric_limits<mapsize_t>::max();
			current_node.parent_y = std::numeric_limits<mapsize_t>::max();
			current_node.color = Color::BLACK;
			current_node.costFromSrc = std::numeric_limits<decltype(current_node.costFromSrc)>::infinity();
		}
	}
}

void WeightMap::dijkstrasMakeMoves(const DijkstrasMove *moves, const size_t numMoves, const Node &currentNode, WeightMap::queue_t &q)
{
	for (size_t i = 0; i < numMoves; i++)
	{

		const int32_t xidx = currentNode.x + moves[i].dx;
		const int32_t yidx = currentNode.y + moves[i].dy;

		// Bounds Check
		if (isValidPoint(xidx, yidx))
		{
			Node &neighborNode = arr[(size_t)xidx][(size_t)yidx];

			// Basic Turning
			fweight_t nn_new_cost_from_src = (moves[i].weight_multiplier * (((fweight_t)(neighborNode.weight + currentNode.weight)) / 2)) + currentNode.costFromSrc;

			// Add cost to turning
			if (isValidPoint(currentNode.parent_x, currentNode.parent_y))
			{
				int32_t parrent_current_dx = currentNode.parent_x - currentNode.x;
				int32_t parrent_current_dy = currentNode.parent_y - currentNode.y;

				if (parrent_current_dx != moves[i].dx || parrent_current_dy != moves[i].dy)
				{
					nn_new_cost_from_src += 5;
				}
			}

			if ((nn_new_cost_from_src < neighborNode.costFromSrc))
			{

				neighborNode.color = Color::RED;
				neighborNode.costFromSrc = nn_new_cost_from_src;
				neighborNode.parent_x = currentNode.x;
				neighborNode.parent_y = currentNode.y;
				q.push(&neighborNode);
			}
		}
	}
}

#include "map_util.hpp"
fweight_t WeightMap::get_linear_cost(Node &a, Node &b)
{
	const int32_t dx = a.x - b.x;
	const int32_t dy = a.y - b.y;

	const mapsize_t top = std::min(a.y, b.y);
	const mapsize_t bottom = std::max(a.y, b.y);

	const mapsize_t left = std::min(a.x, b.x);
	const mapsize_t right = std::max(a.x, b.x);

	if (dx == 0)
	{
		fweight_t weight = (0.5 * a.weight) + (0.5 * b.weight);
		for (mapsize_t i = top; i < bottom; i++)
		{
			weight += arr[a.x][i].weight;
		}
		return weight;
	}
	else if (dy == 0)
	{
		fweight_t weight = (0.5 * a.weight) + (0.5 * b.weight);
		for (mapsize_t i = left; i < right; i++)
		{
			weight += arr[i][a.y].weight;
		}
		return weight;
	}
	else
	{
		// Set up the line to run through the center of a
		const float m = (float)dy / (float)dx;
		const float h = (a.y + 0.5f) - (m * (a.x + 0.5f));
		float weight = 0;

		// Reset colors
		for (size_t x = left; x <= right; x++)
		{
			for (size_t y = top; y <= bottom; y++)
			{
				arr[x][y].color = Color::BLACK;
			}
		}

		float linear_length = 0;

		std::queue<std::reference_wrapper<Node>> nodes;
		nodes.emplace(a);
		while (!nodes.empty())
		{
			Node &n = nodes.front();
			nodes.pop();

			// If node is in box
			if (n.x >= left && n.x <= right && n.y >= top && n.y <= bottom)
			{
				if (n.color != Color::BLACK)
				{
					continue;
				}

				const float sq_x = n.x + 0.5;
				const float sq_y = n.y + 0.5;
				const float line_length = get_length_line_in_square(sq_x, sq_y, m, h);
				linear_length += line_length;
				weight += n.weight * line_length;
				n.color = Color::GREEN;

				// Only push neighbors if the line goes through this box
				if (line_length > 0)
				{
					for (auto &move : WeightMap::moves)
					{
						const mapsize_t x = n.x + move.dx;
						const mapsize_t y = n.y + move.dy;
						if (isValidPoint(x, y))
						{
							nodes.emplace(arr[x][y]);
						}
					}
				}
			}
		}

		//linear_length -= (get_length_line_in_square(a.x + 0.5, a.y + 0.5, m, h) + get_length_line_in_square(b.x + 0.5, b.y + 0.5, m, h)) / 2;

		weight -= (get_length_line_in_square(a.x + 0.5, a.y + 0.5, m, h) * a.weight) / 2;
		weight -= (get_length_line_in_square(b.x + 0.5, b.y + 0.5, m, h) * b.weight) / 2;

		return weight;
	}
}

fweight_t WeightMap::distance(fweight_t x1, fweight_t y1, fweight_t x2, fweight_t y2)
{
	return (fweight_t)std::sqrt(((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2)));
}

/*
Returns the ratio of the path through these three points compared to the path between the end points
*/
fweight_t WeightMap::length_ratio(WeightMap::point_t &l, WeightMap::point_t &m, WeightMap::point_t &r)
{
	return distance(l.first, l.second, r.first, r.second) / (distance(l.first, l.second, m.first, m.second) + distance(m.first, m.second, r.first, r.second));
}


//Weight Map Serialization / Deserialization

#if defined(_MSC_VER) || defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
   #include <winsock.h>
#elif __linux__
    #include <arpa/inet.h>
#else
#   error "Unknown compiler"
#endif



std::pair<const char*, const size_t> WeightMap::serialize() const{
	static_assert((sizeof(mapsize_t) == sizeof(weight_t)) && (sizeof(weight_t) == 2), "Current Serialization assumes 2 byte mapsize_t and weight_t due to htonl");
	const size_t buff_size_bytes = (sizeof(mapsize_t) * 2) + (sizeof(weight_t) * this->width * this->height);
	
	char* const buff = (char*)malloc(buff_size_bytes);
	if (buff == nullptr){
		throw std::bad_alloc();
	}

	reinterpret_cast<mapsize_t*>(buff)[0]= htons(this->width);
	reinterpret_cast<mapsize_t*>(buff)[1] = htons(this->height);

	weight_t* const weight_buff = (weight_t*)&(((mapsize_t*) buff)[2]);
	for (size_t y = 0; y < this->height; y++){
		for (size_t x = 0; x <  this-> width; x++){
			weight_buff[(y * this->width) + x] = htons(arr[x][y].weight);
		}
	}

	return std::make_pair(buff, buff_size_bytes);
}

WeightMap WeightMap::deserialize(std::pair<const char*, const size_t> bytes){
	if (bytes.second <= 4){
		throw std::invalid_argument("Not enough bytes to construct a WeightMap");
	}


	const char* const buff = bytes.first;

	const mapsize_t* const map_buff = (const mapsize_t*)buff;

	const mapsize_t width = ntohs(map_buff[0]);
	const mapsize_t height = ntohs(map_buff[1]);

	const size_t expected_buffer_size = (sizeof(mapsize_t) * 2) + (sizeof(weight_t) * width * height);
	if (bytes.second < expected_buffer_size){
		throw std::invalid_argument("Not enough bytes to construct a WeightMap of given size");
	}

	WeightMap wm(width, height);

	const weight_t* const weight_buff = (const weight_t*)&(map_buff[2]);
	for (size_t y = 0; y < height; y++){
		for (size_t x = 0; x <  width; x++){
			wm.arr[x][y].weight = ntohs(weight_buff[(y * width) + x]);
		}
	}

	return wm;
}

//---------------	Node Methods	----------------------------
bool WeightMap::Node::operator==(const WeightMap::Node &other)
{
	auto b = other;
	auto a = *this;
	return this->color == b.color && a.costFromSrc == b.costFromSrc && a.parent_x == b.parent_x && a.parent_y == b.parent_y && a.weight == b.weight && a.x == b.x && a.y == b.y;
}

std::ostream &operator<<(std::ostream &os, const WeightMap::Color &c)
{
	switch (c)
	{
	case WeightMap::Color::BLACK:
		os << "BLACK";
		break;
	case WeightMap::Color::RED:
		os << "RED";
		break;
	case WeightMap::Color::GREEN:
		os << "GREEN";
		break;
	default:
		os << "UNKNOWN";
		break;
	}

	return os;
}

std::ostream& operator<<(std::ostream &os, const WeightMap::Node &n)
{
	os << "{ X: " << n.x << "; Y: " << n.y << "; Weight: " << n.weight << "; Parent: (" << n.parent_x << ',' << n.parent_y << "); Color: " << n.color << "; CostFromSrc: " << n.costFromSrc << "}";
	return os;
}

std::ostream &operator<<(std::ostream &os, const WeightMap &wm)
{
	for (size_t y = 0; y < wm.height; y++)
	{
		for (size_t x = 0; x < wm.width; x++)
		{
			os << wm.arr[x][y] << ' ';
		}
		os << '\n';
	}

	return os;
}

bool WeightMap::NodeCmp::operator()(const Node &a, const Node &b)
{
	return a.costFromSrc > b.costFromSrc;
}

bool WeightMap::NodeCmp::operator()(const Node *a, const Node *b)
{
	return a->costFromSrc > b->costFromSrc;
}
