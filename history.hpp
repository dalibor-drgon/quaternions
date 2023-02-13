
#include <deque>

template<class T, size_t size>
class History {
public:
    std::deque<T> list;

    void add(const T & val) {
        if (list.size() == size) list.pop_front();
        list.push_back(val);
    }

    T average(T zero) {
        T avg = zero;
        for (T & val : list) {
            avg += val;
        }
        avg *= (1.0f / list.size());
        return avg;
    }
};
