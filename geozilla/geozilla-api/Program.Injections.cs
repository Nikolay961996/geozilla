using geozilla_bl.Services.Generation.Abstract;
using geozilla_bl.Services.Generation.Concrete;

namespace geozilla_api;

public static class ProgramInjections
{
    public static WebApplicationBuilder AddGeozillaDependencyInjections(this WebApplicationBuilder builder)
    {
        builder.Services.AddScoped<IGeoJsonService, GeoJsonService>();

        return builder;
    }
}
