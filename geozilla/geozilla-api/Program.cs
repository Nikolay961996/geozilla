using geozilla_api;

var builder = WebApplication.CreateBuilder(args);

builder.AddGeozillaDependencyInjections();
builder.AddGeozillaEndpoints();
builder.AddGeozillaSwagger();

var app = builder.Build();

app.UseGeozillaSwagger();
app.UseGeozillaEndpoints();

app.Run();
