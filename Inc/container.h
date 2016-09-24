
#ifndef _CONTAINER
#define _CONTAINER

#ifdef __cplusplus
class container {
    protected:
        int count;
    public:
        ~container();
        container();

        void loop();
};

#endif
#ifdef __cplusplus
extern "C" {
#endif
struct container *newClass();
// struct Test *newTest();
void runLoop(struct container *);
// void runTest(struct Test *);
#ifdef __cplusplus
}
#endif

#endif

