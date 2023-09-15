#include <iostream>
#include <vector>
#include <utility>
#include <tuple>

std::pair<size_t, size_t> find_biggest_gap(const std::vector<float>& data)
{
    size_t gap_start = 0;
    size_t max_gap_start = 0;
    size_t max_gap_size = 0;
    size_t current_gap_size = 0;
    for (size_t i = 0; i < data.size(); ++i)
    {
        if (data[i] > 1.5) 
        {
            if (current_gap_size == 0)
            {
                gap_start = i;
            }
            current_gap_size++;
        }
        else
        {
            if (current_gap_size > max_gap_size)
            {
                max_gap_size = current_gap_size;
                max_gap_start = gap_start;
            }
            current_gap_size = 0;
        }
    }
    return std::make_pair(max_gap_start, max_gap_start + max_gap_size);
}

int main()
{
    std::vector<float> data = {0.5, 2.0, 3.0, 0.8, 0.9, 1.0, 2.5, 2.6, 2.7, 0.3, 0.2, 4.0};

    std::pair<size_t, size_t> gap_indices = find_biggest_gap(data);

    size_t gap_start, gap_end;
    std::tie(gap_start, gap_end) = gap_indices;

    std::cout << "Largest Gap Start Index: " << gap_start << std::endl;
    std::cout << "Largest Gap End Index: " << gap_end << std::endl;

    return 0;
}