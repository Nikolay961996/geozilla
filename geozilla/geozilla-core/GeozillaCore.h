#pragma once

__declspec(dllexport) const char* GenerateGeoJson(const char* pathToB3dm);
__declspec(dllexport) void FreeBuffer(const char* buffer);
