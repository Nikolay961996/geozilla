namespace geozilla_api;

public static class ProgramCors
{
    private const string _allowAllPolicy = "_stupidAllowFuckingSpecificOrigins";

    public static WebApplicationBuilder AddGeozillaCors(this WebApplicationBuilder builder)
    {
        builder.Services.AddCors(options =>
        {
            options.AddPolicy(name: _allowAllPolicy,
                              policy =>
                              {
                                  policy
                                    .SetIsOriginAllowed(origin => true)
                                    .AllowAnyHeader()
                                    .AllowAnyMethod()
                                    .AllowCredentials();
                              });
        });

        return builder;
    }

    public static IApplicationBuilder UseGeozillaCors(this IApplicationBuilder app)
    {
        app.UseCors(_allowAllPolicy);

        return app;
    }
}
