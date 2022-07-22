//test.hpp

#ifndef TEST_HPP
#define TEST_HPP

template <class T> class testClass
{
	public:
		testClass();
		~testClass();
	private:
		void testFunction(T& data);
};

#endif //TEST_HPP