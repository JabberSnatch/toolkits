#define VOXTK_IMPLEMENTATION
#include "voxtk.hh"

voxtk::VoxelField::Node* EmplaceLeafTest(voxtk::VoxelField& field, numtk::vec3i const& point) {
    return field.EmplaceLeaf(field.LookupNode(point), point);
}

int main(int argc, char const** argv)
{
    {
        voxtk::VoxelField test_field = {};
        test_field.Set({ 0, 0, 0 }, true);
        test_field.Set({ 32, 32, 32 }, true);
        test_field.SetVolume({{ -16, -16, -16 }, { 16, 16, 16 }}, true );
        test_field.SetVolume({{ -15, -15, -15 }, { 15, 15, 15 }}, true );

        test_field;
    }

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
        EmplaceLeafTest(test_field, { -1, -1, -1 });
        EmplaceLeafTest(test_field, { 1, 1, 1 });

        for (uint32_t index = 1; index < 10; ++index)
        {
            int32_t offset = -(1 << (voxtk::VoxelMask::kLogSize * index));
            voxtk::VoxelField::Node const* leaf0 =
                EmplaceLeafTest(test_field, numtk::vec3i::Constant(offset-1));
            voxtk::VoxelField::Node const* leaf1 =
                EmplaceLeafTest(test_field, numtk::vec3i::Constant(offset*2-1));
        }

        test_field;
    }

    {
        voxtk::VoxelField test_field = {};
        EmplaceLeafTest(test_field, { 0, 0, 0 });
        EmplaceLeafTest(test_field, { -1, -1, -1 });
        EmplaceLeafTest(test_field, { 4095, 4095, 4095 });

        test_field;
    }

    {
        voxtk::VoxelField test_field = {};
        EmplaceLeafTest(test_field, { -1, -1, -1 });
        EmplaceLeafTest(test_field, { 16, 16, 16 });
        EmplaceLeafTest(test_field, { -4096, -4096, -4096 });

        test_field;
    }

    {
        voxtk::VoxelField test_field = {};
        EmplaceLeafTest(test_field, { 0, 0, 0 });
        EmplaceLeafTest(test_field, { 511, 511, 511 });
        EmplaceLeafTest(test_field, { -512, -512, -512 });
        EmplaceLeafTest(test_field, { 4095, 4095, 4095 });
        EmplaceLeafTest(test_field, { -262144, -262144, -262144 });

        test_field;
    }

    {
        voxtk::VoxelField test_field = {};
        EmplaceLeafTest(test_field, { 4096, 0, 0 });
        EmplaceLeafTest(test_field, { -4096, 0, 512 });
        EmplaceLeafTest(test_field, { 0, -262144, 512 });
        EmplaceLeafTest(test_field, { 0, 262144, -4096 });

        test_field;
    }

    return 0;
}
