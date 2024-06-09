using geozilla_api.Endpoints.GeneratorGeoJson;

namespace geozilla_api;

public static class ProgramEndpoints
{
    public static WebApplicationBuilder AddGeozillaEndpoints(this WebApplicationBuilder builder)
    {
        builder.Services.AddEndpointsApiExplorer();

        return builder;
    }

    public static IEndpointRouteBuilder UseGeozillaEndpoints(this IEndpointRouteBuilder builder)
    {
        builder.AddGeneratoGeoJsonApi();

        return builder;
    }
}
