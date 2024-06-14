namespace geozilla_bl.Services.Generation.Abstract;

public interface IGeoJsonService
{
    Task<string> Generate(string path);
}
