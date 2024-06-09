using geozilla_bl.Services.Generation.Abstract;

namespace geozilla_bl.Services.Generation.Concrete;

public class GeoJsonService: IGeoJsonService
{
    public async Task<string> Generate(string pathToB3dm)
    {
        return await Task.FromResult("{ geojson }");
    }
}
