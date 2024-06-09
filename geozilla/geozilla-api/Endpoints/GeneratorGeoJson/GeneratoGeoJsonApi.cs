using geozilla_bl.Services.Generation.Abstract;

namespace geozilla_api.Endpoints.GeneratorGeoJson;

public static class GeneratoGeoJsonApi
{
    public static IEndpointRouteBuilder AddGeneratoGeoJsonApi(this IEndpointRouteBuilder builder)
    {
        builder.MapPost("generate/geo-json", GenerateGeoJson)
            .WithName("GenerateGeoJson").WithOpenApi();

        return builder;
    }

    private static async Task<string> GenerateGeoJson(IGeoJsonService service)
    {
        var result = await service.Generate("D://path/from/request");

        return result;
    }
}
