namespace geozilla_api;

public static class ProgramSwagger
{
    public static WebApplicationBuilder AddGeozillaSwagger(this WebApplicationBuilder builder)
    {
        builder.Services.AddSwaggerGen();

        return builder;
    }

    public static WebApplication UseGeozillaSwagger(this WebApplication app)
    {
        if (app.Environment.IsDevelopment())
        {
            app.UseSwagger();
            app.UseSwaggerUI();
        }

        return app;
    }
}
