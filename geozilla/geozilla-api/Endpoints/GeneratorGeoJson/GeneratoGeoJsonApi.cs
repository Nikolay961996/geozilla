namespace geozilla_api.Endpoints.GeneratorGeoJson;

public static class GeneratoGeoJsonApi
{
    public static IEndpointRouteBuilder AddGeneratoGeoJsonApi(this IEndpointRouteBuilder builder)
    {
        builder.MapPost("generate/geo-json", GenerateGeoJson)
            .WithName("GenerateGeoJson").WithOpenApi();

        return builder;
    }

    private static string GenerateGeoJson()
    {
        return "Hi";
    }
}
