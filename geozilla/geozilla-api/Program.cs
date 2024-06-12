using geozilla_api;

var builder = WebApplication.CreateBuilder(args);

builder.AddGeozillaCors();
builder.AddGeozillaDependencyInjections();
builder.AddGeozillaEndpoints();
builder.AddGeozillaSwagger();

var app = builder.Build();

app.UseGeozillaCors();
app.UseGeozillaSwagger();
app.UseGeozillaEndpoints();

app.Run();
