using Microsoft.AspNetCore.Components.Web;
using Microsoft.AspNetCore.Components.WebAssembly.Hosting;
using PoRobotTrain.Client;
using PoRobotTrain.Client.Services;
using FluentValidation;
using PoRobotTrain.Shared.Validation;
using PoRobotTrain.Shared.Contracts;
using Radzen;

var builder = WebAssemblyHostBuilder.CreateDefault(args);
builder.RootComponents.Add<App>("#app");
builder.RootComponents.Add<HeadOutlet>("head::after");

builder.Services.AddScoped(_ => new HttpClient
{
    BaseAddress = new Uri(builder.HostEnvironment.BaseAddress)
});

builder.Services.AddRadzenComponents();
builder.Services.AddScoped<RobotApiClient>();
builder.Services.AddScoped<ConsoleApiClient>();
builder.Services.AddScoped<UiState>();
builder.Services.AddScoped<TwinKinematicsService>();

// Validators by type, not by assembly scan: the trimmed Release build keeps exactly these constructors.
builder.Services.AddScoped<IValidator<MotionClipDto>, MotionClipValidator>();
builder.Services.AddScoped<IValidator<ConnectionProfileDto>, ConnectionProfileValidator>();
builder.Services.AddScoped<IValidator<WalkStartRequest>, WalkStartRequestValidator>();
builder.Services.AddScoped<IValidator<WalkThrowRequest>, WalkThrowRequestValidator>();
builder.Services.AddScoped<IValidator<WalkReplayRequest>, WalkReplayRequestValidator>();
builder.Services.AddScoped<IValidator<WalkFilterRequest>, WalkFilterRequestValidator>();
builder.Services.AddScoped<IValidator<TaughtPoseRequest>, TaughtPoseRequestValidator>();
builder.Services.AddScoped<IValidator<GapRequest>, GapRequestValidator>();
builder.Services.AddScoped<IValidator<DuelRequest>, DuelRequestValidator>();

await builder.Build().RunAsync();
