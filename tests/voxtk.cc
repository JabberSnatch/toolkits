#define VOXTK_IMPLEMENTATION
#include "voxtk.hh"

int main(int argc, char const** argv)
{
    voxtk::VoxelField test_field = {};
    test_field.EmplaceLeaf({ -1, -1, -1 });
    test_field.EmplaceLeaf({ 1, 1, 1 });
    test_field.EmplaceLeaf({ -9, -9, -9 });
    return 0;
}
