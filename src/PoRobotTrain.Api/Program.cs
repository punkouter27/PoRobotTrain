using Microsoft.Extensions.Options;
using PoRobotTrain.Api.Features.Diagnostics;
using PoRobotTrain.Api.Features.RobotControl;
using Scalar.AspNetCore;
using Serilog;

Log.Logger = new LoggerConfiguration()
    .WriteTo.Console(formatProvider: System.Globalization.CultureInfo.InvariantCulture)
    .CreateBootstrapLogger();

try
{
    var builder = WebApplication.CreateBuilder(args);

    if (builder.Environment.IsEnvironment("Test"))
    {
        builder.WebHost.UseStaticWebAssets();
    }

    builder.Host.UseSerilog((ctx, services, cfg) => cfg
        .ReadFrom.Configuration(ctx.Configuration)
        .ReadFrom.Services(services)
        .Enrich.FromLogContext()
        .WriteTo.Console(formatProvider: System.Globalization.CultureInfo.InvariantCulture));

    builder.Services.AddProblemDetails();
    builder.Services.AddOpenApi(options =>
        options.OpenApiVersion = Microsoft.OpenApi.OpenApiSpecVersion.OpenApi3_1);

    builder.Services.Configure<RobotServiceOptions>(
        builder.Configuration.GetSection(RobotServiceOptions.SectionName));

    builder.Services.AddHttpClient<IRobotControlClient, RobotControlHttpClient>((sp, client) =>
    {
        var opts = sp.GetRequiredService<IOptions<RobotServiceOptions>>().Value;
        client.BaseAddress = new Uri(opts.BaseUrl);
        client.Timeout = RobotServiceTimeoutHandler.Ceiling; // per request: 5 s, longer for the skill tools (handler below)
        client.DefaultRequestHeaders.ConnectionClose = true; // Python BaseHTTPRequestHandler closes HTTP/1.0 sockets per response
    }).AddHttpMessageHandler(() => new RobotServiceTimeoutHandler());

    var app = builder.Build();

    app.UseExceptionHandler(new ExceptionHandlerOptions
    {
        StatusCodeSelector = ex => ex is BadHttpRequestException bad
            ? bad.StatusCode
            : StatusCodes.Status500InternalServerError
    });

    if (app.Environment.IsDevelopment() || app.Environment.IsEnvironment("Test"))
    {
        app.MapOpenApi();
        app.MapScalarApiReference("/scalar");
    }

    app.Use(async (ctx, next) =>
    {
        if (ctx.Request.Path.StartsWithSegments("/_framework") || ctx.Request.Path.StartsWithSegments("/css") || ctx.Request.Path == "/")
        {
            ctx.Response.Headers.CacheControl = "no-cache, must-revalidate";
        }
        await next();
    });

    app.UseBlazorFrameworkFiles();
    // Browser twin assets (wwwroot/twin/, built by scripts/build_web_twin.py) use types the default provider lacks.
    var contentTypes = new Microsoft.AspNetCore.StaticFiles.FileExtensionContentTypeProvider();
    contentTypes.Mappings[".glb"] = "model/gltf-binary";
    contentTypes.Mappings[".onnx"] = "application/octet-stream";
    app.UseStaticFiles(new StaticFileOptions { ContentTypeProvider = contentTypes });
    app.UseRouting();

    app.MapDiagEndpoints();
    app.MapRobotEndpoints();

    app.MapStaticAssets();
    app.MapFallbackToFile("index.html");

    app.Run();
}
catch (Exception ex) when (ex is not HostAbortedException)
{
    Log.Fatal(ex, "PoRobotTrain.Api failed to start");
    throw;
}
finally
{
    Log.CloseAndFlush();
}

public partial class Program;
