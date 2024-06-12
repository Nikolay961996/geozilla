using System.Text.Json.Serialization;

namespace geozilla_api.Endpoints.GeneratorGeoJson.Requests;

public class GenerateGeoJsonRequest
{
    [JsonPropertyName("latitude")]
    public float Latitude { get; set; }

    [JsonPropertyName("longitude")]
    public float Longitude { get; set; }

    [JsonPropertyName("file")]
    public required IFormFile File { get; set; }
}
