#ifndef PATH_TRACKER_ARRAY_PARSER_HPP_
#define PATH_TRACKER_ARRAY_PARSER_HPP_

#include <sstream>
#include <string>
#include <vector>

// Reference from
// https://github.com/ros-planning/navigation2/blob/main/nav2_costmap_2d/src/array_parser.cpp

namespace path_tracker {

    class ArrayParser {
    public:
        static std::vector<std::vector<float>> parseVVF(const std::string& input,
                                                        std::string& error_return)
        {
            std::vector<std::vector<float>> result;

            std::stringstream input_ss(input);
            int depth = 0;
            std::vector<float> current_vector;
            while (!!input_ss && !input_ss.eof()) {
                switch (input_ss.peek()) {
                    case EOF:
                        break;
                    case '[':
                        depth++;
                        if (depth > 2) {
                            error_return = "Array depth greater than 2";
                            return result;
                        }
                        input_ss.get();
                        current_vector.clear();
                        break;
                    case ']':
                        depth--;
                        if (depth < 0) {
                            error_return = "More close ] than open [";
                            return result;
                        }
                        input_ss.get();
                        if (depth == 1) {
                            result.push_back(current_vector);
                        }
                        break;
                    case ',':
                    case ' ':
                    case '\t':
                        input_ss.get();
                        break;
                    default: // All other characters should be part of the numbers.
                        if (depth != 2) {
                            std::stringstream err_ss;
                            err_ss << "Numbers at depth other than 2. Char was '"
                                   << char(input_ss.peek()) << "'.";
                            error_return = err_ss.str();
                            return result;
                        }
                        float value;
                        input_ss >> value;
                        if (!!input_ss) {
                            current_vector.push_back(value);
                        }
                        break;
                }
            }

            if (depth != 0) {
                error_return = "Unterminated vector string.";
            }
            else {
                error_return = "";
            }

            return result;
        }
    };

} // namespace path_tracker

#endif