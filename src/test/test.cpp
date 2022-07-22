//test.cpp

#include <pointcloud_utils/test/test.hpp>
#include <iostream>

template <class T> testClass<T>::testClass()
{
	std::cout << "Starting test class\n";
	T test_data;
	testFunction(test_data);
}

template <class T> testClass<T>::~testClass()
{

}

template <class T> void testClass<T>::testFunction(T& data)
{
	std::cout << "Data recieved\n";
}