using geozilla_api.Endpoints.GeneratorGeoJson.Requests;
using geozilla_bl.Services.Generation.Abstract;
using Microsoft.AspNetCore.Mvc;

namespace geozilla_api.Endpoints.GeneratorGeoJson;

public static class GeneratoGeoJsonApi
{
    public static IEndpointRouteBuilder AddGeneratoGeoJsonApi(this IEndpointRouteBuilder builder)
    {
        builder.MapPost("generate/geo-json", GenerateGeoJson)
            .WithName("GenerateGeoJson").WithOpenApi()
            .DisableAntiforgery();

        return builder;
    }

    private static async Task<string> GenerateGeoJson([FromForm] GenerateGeoJsonRequest request, IGeoJsonService service)
    {
        Console.WriteLine($"lat: {request.Latitude}; lng: {request.Longitude}");
        Console.WriteLine($"file: {request.File.FileName}");


        var result = await service.Generate("D://path/from/request");

        return result;
    }
}
