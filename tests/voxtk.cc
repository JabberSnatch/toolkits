#define VOXTK_IMPLEMENTATION
#include "voxtk.hh"

int main(int argc, char const** argv)
{
    {
        voxtk::VoxelField test_field = {};
        test_field.Set({ 1, 1, 1 }, true);
        test_field.Set({ -1, -1, -1 }, true);
        test_field.Set({ 4095, 4095, 4095 }, true);

        test_field.Test({ 1, 1, 1 });
        test_field.Test({ 16, 16, 16 });
        test_field.Test({ 4095, 4095, 4095 });

        test_field;
    }

    {
        voxtk::VoxelField test_field = {};
        test_field.EmplaceLeaf({ -1, -1, -1 });
        test_field.EmplaceLeaf({ 1, 1, 1 });

        for (uint32_t index = 1; index < 9; ++index)
        {
            int32_t offset = -(1 << (voxtk::VoxelMask::kLogSize * index));
            voxtk::VoxelField::Node const* leaf0 =
                test_field.EmplaceLeaf(numtk::vec3i::Constant(offset-1));
            voxtk::VoxelField::Node const* leaf1 =
                test_field.EmplaceLeaf(numtk::vec3i::Constant(offset*2-1));
        }

        test_field;
    }

    {
        voxtk::VoxelField test_field = {};
        test_field.EmplaceLeaf({ 0, 0, 0 });
        test_field.EmplaceLeaf({ -1, -1, -1 });
        test_field.EmplaceLeaf({ 4095, 4095, 4095 });

        test_field;
    }

    {
        voxtk::VoxelField test_field = {};
        test_field.EmplaceLeaf({ -1, -1, -1 });
        test_field.EmplaceLeaf({ 16, 16, 16 });
        test_field.EmplaceLeaf({ -4096, -4096, -4096 });

        test_field;
    }

    {
        voxtk::VoxelField test_field = {};
        test_field.EmplaceLeaf({ 0, 0, 0 });
        test_field.EmplaceLeaf({ 511, 511, 511 });
        test_field.EmplaceLeaf({ -512, -512, -512 });
        test_field.EmplaceLeaf({ 4095, 4095, 4095 });
        test_field.EmplaceLeaf({ -262144, -262144, -262144 });

        test_field;
    }

    {
        voxtk::VoxelField test_field = {};
        test_field.EmplaceLeaf({ 4096, 0, 0 });
        test_field.EmplaceLeaf({ -4096, 0, 512 });
        test_field.EmplaceLeaf({ 0, -262144, 512 });
        test_field.EmplaceLeaf({ 0, 262144, -4096 });

        test_field;
    }

    return 0;
}
