#include <vector>
#include <iostream>
#include <iomanip>
#include <string>
struct item
{
    double value;
    int capacity;
};
class Knapsack
{
public:
    Knapsack(double knapsack_size, int item_size, int min_width, int min_height);
    ~Knapsack();
    void add_items(double value, int capacity);
    void clear_items();
    
    double  solve();
    void get_items_selected(std::vector<item>& resultItems, std::vector<int>& resultItemsIndices);
    friend std::ostream &operator <<( std::ostream&, const Knapsack& );
private:
    void init_knapsack_table();
    std::vector<item> m_items;
    int m_knapsack_size;
    int m_item_size;
    // 2D matrix to store intermediate
    // result of the knapsack calculation.
    std::vector<std::vector<double> > m_knapsack_table;
    // 2D matrix to store the intermediate
    // result for which item is selected
    // Later this matrix is used to get which items
    // are selected for optimal calculation.
    std::vector<std::vector<double> > m_selection_table;

    int current_n_items;
};
