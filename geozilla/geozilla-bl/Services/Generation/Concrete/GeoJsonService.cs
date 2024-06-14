using geozilla_bl.Services.Generation.Abstract;
using System.Runtime.InteropServices;

namespace geozilla_bl.Services.Generation.Concrete;

public class GeoJsonService: IGeoJsonService
{
    public async Task<string> Generate(string path)
    {
        IntPtr geoJsonPtr = GeozillaCoreDll.GenerateGeoJson(path);
        string? geoJson = Marshal.PtrToStringAnsi(geoJsonPtr);
        GeozillaCoreDll.FreeBuffer(geoJsonPtr);
        return await Task.FromResult(geoJson ?? "{}");
    }
}
