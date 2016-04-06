#include "knapsack.hpp"

Knapsack::Knapsack(double knapsack_size, int item_size, int min_width, int min_height):
        m_knapsack_size(knapsack_size),
        m_item_size(item_size),
        m_knapsack_table(item_size + 1,std::vector<double>(knapsack_size + 1)),
        m_selection_table(item_size + 1,std::vector<double>(knapsack_size + 1)),
        current_n_items(0)    
    {
        m_items.reserve(m_item_size);
    }
        
    void Knapsack::add_items(double value, int capacity) 
    {
        item t;
        t.value = value;
        t.capacity = capacity;
        m_items.push_back(t);
        ++current_n_items;
    }
    
    void Knapsack::clear_items()
    {
        m_items.clear();
        current_n_items=0;
    }
    
    double Knapsack::solve() 
    {
        // Initialize the first row in both the
        // tables, these values are used as default
        // as if no items are selected and no capacity
        // is available.
        // This is the default case for bottom-up approach.
        for ( int i = 0; i < m_knapsack_size + 1 ; i++)
        {
            m_knapsack_table [0][i]  = 0;
            m_selection_table[0][i]  = 0;
        }
        
//         std::cout << "ITEMS:" << current_n_items<< " of "<< m_item_size<<std::endl;
//         
//         if(current_n_items>m_items.size())
//         {
//             std::cout << "foda-se"<< std::endl;
//         }
        int row = 1;
        for ( std::vector<item>::iterator itemIterator = m_items.begin(); itemIterator != m_items.end(); ++itemIterator) 
        {
            if(row>=current_n_items)
                break;
            item currentItem = *itemIterator;
            int col = 0; // col is capacity available.
            while ( col < m_knapsack_size + 1) 
            {
                //1. Check if item can be fit by it's own.
                if ( currentItem.capacity > col ) 
                {   
                    //2. Get the solution for the already solved
                    //   knapsack problem.
                    m_knapsack_table[row][col] = m_knapsack_table[row - 1][col];
                    // Update the selection table as we are not able to accomodate this item
                    // eventually we are not selecting this ite.
                    m_selection_table[row][col] = 0;
                }
                else 
                {
                    // We are now considering the item.
                    int capacity_remaining = col - currentItem.capacity;
                    double new_value  = currentItem.value + m_knapsack_table[row - 1][capacity_remaining];
                    double prev_value = m_knapsack_table[row - 1][col];
                    if ( prev_value >= new_value) 
                    {
                        // There is no gain here to consider this item.
                        m_knapsack_table[row][col] = m_knapsack_table[row - 1][col];
                        m_selection_table[row][col] = 0;
                    } 
                    else 
                    {
                        // Add this item into the knapsack.
                        m_knapsack_table[row][col]  = new_value;
                        // Update the selection table as we are considering
                        // this item.
                        m_selection_table[row][col] = 1;
                    }
                }
                ++col;
            }
            ++row;
        }
        return m_knapsack_table[current_n_items][m_knapsack_size];
    }
    void Knapsack::get_items_selected(std::vector<item>& resultItems, std::vector<int>& resultItemsIndices) 
    {
        int row = current_n_items;
        int col = m_knapsack_size;
        int cap = m_knapsack_size;
        while ( cap > 0 && row>=0) 
        {
            if ( m_selection_table[row][col] == 1) 
            {
                resultItems.push_back(m_items[row - 1]);
                resultItemsIndices.push_back(row-1);
                cap = cap - m_items[row - 1].capacity;
                col = cap;
            }
            row = row - 1;
        }
    }
    std::ostream &operator << (std::ostream &out, const Knapsack &knp ) 
    {
        out << std::endl;
        out << "SOLUTION MATRIX" << std::endl << std::endl;
        out << std::setw(15) << "Capacity |";
        for ( int i = 0; i <= knp.m_knapsack_size; i++) {
            out << std::setw(5) << i ;
        }
        out << std::endl;
        out << std::endl;
        int row = 0;
        out << std::setw(15) << "NONE |";
        int col = 0;
        while ( col < knp.m_knapsack_size + 1 ) {
            out << std::setw(5) << knp.m_knapsack_table[row][col];
            col++;
        }
        out << std::endl;
        row++;
        for ( std::vector<item>::const_iterator itemIterator = (knp.m_items).begin();
        itemIterator != knp.m_items.end();
        ++itemIterator) {
            out << "(V:"
                    << std::setw(2)
                    << itemIterator->value
                    << ", "
                    << "W:"
                    << itemIterator->capacity
                    << ")"
                    << std::setw(4)
                    << "|" ;
            col = 0;
            while ( col < knp.m_knapsack_size + 1 ) {
                out << std::setw(5) << knp.m_knapsack_table[row][col];
                col++;
            }
            row++;
            out << std::endl;
        }
        out << std::endl;
        out << "SELECTION MATRIX" << std::endl << std::endl;
        out << std::setw(15) << "Capacity |";
        for ( int i = 0; i <= knp.m_knapsack_size; i++) {
            out << std::setw(5) << i ;
        }
        out << std::endl;
        out << std::endl;
        row = 0;
        out << std::setw(15) << "NONE |";
        col = 0;
        while ( col < knp.m_knapsack_size + 1 ) {
            out << std::setw(5) << knp.m_knapsack_table[row][col];
            col++;
        }
        out << std::endl;
        row++;
        for ( std::vector<item>::const_iterator itemIterator = (knp.m_items).begin();
        itemIterator != knp.m_items.end();
        ++itemIterator) {
            out << "(V:"
                    << std::setw(2)
                    << itemIterator->value
                    << ", "
                    << "W:"
                    << itemIterator->capacity
                    << ")"
                    << std::setw(4)
                    << "|" ;
            col = 0;
            while ( col < knp.m_knapsack_size + 1 ) {
                out << std::setw(5) << knp.m_selection_table[row][col];
                col++;
            }
            row++;
            out << std::endl;
        }
        return out;
    }
    Knapsack::~Knapsack() {
    }